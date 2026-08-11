//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalTransportService.cc
//---------------------------------------------------------------------------//
#include "OpticalTransportService.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "corecel/Assert.hh"
#include "corecel/io/Logger.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
struct OpticalTransportService::SharedState
{
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;

    struct EventResult
    {
        LaneId lane;
        std::vector<optical::DetectorHit> hits;
        Clock::time_point registered;
        bool ready{false};
        bool pumping{false};
        bool delivered{false};
    };

    struct LaneDelivery
    {
        long next_ordinal{-1};
        std::unordered_set<long> completed;
    };

    struct LaneMetrics
    {
        size_type staged_bursts{0};
        size_type staged_photons{0};
        size_type idle_parks{0};
        Duration idle_park_time{Duration::zero()};
        size_type mailbox_hits{0};
        size_type hit_mail_high_water{0};
        size_type pump_count{0};
        size_type pump_hits_total{0};
        size_type pump_hits_max{0};
        size_type registered_events{0};
        size_type retired_events{0};
        Duration retirement_time{Duration::zero()};
        Duration retirement_max{Duration::zero()};
        bool started{false};
    };

    SharedState(Options const& opts,
                HitCallback hit_callback_input,
                ActionTimeCallback action_time_callback_input)
        : options(opts)
        , events(opts.num_lanes, opts.unresolved_limit, opts.base_ordinal)
        , ingress(opts.num_lanes)
        , delivery(opts.num_lanes)
        , lane_metrics(opts.num_lanes)
        , hit_callback(std::move(hit_callback_input))
        , action_time_callback(std::move(action_time_callback_input))
        , census_base(opts.base_ordinal)
    {
        auto const num_lanes = static_cast<long>(delivery.size());
        auto const base_lane = opts.base_ordinal % num_lanes;
        for (size_type i = 0; i < delivery.size(); ++i)
        {
            auto const lane = static_cast<long>(i);
            auto const offset = (lane - base_lane + num_lanes) % num_lanes;
            delivery[i].next_ordinal = opts.base_ordinal + offset;
        }
    }

    Options options;
    mutable std::mutex mutex;
    std::mutex pump_mutex;
    std::condition_variable work_cv;
    std::condition_variable state_cv;
    OpticalEventTable events;
    std::vector<std::deque<OpticalTransportLaneCommand>> ingress;
    std::vector<LaneDelivery> delivery;
    std::vector<LaneMetrics> lane_metrics;
    std::unordered_map<long, EventResult> results;
    HitCallback hit_callback;
    ActionTimeCallback action_time_callback;
    std::exception_ptr terminal_error;
    std::atomic<long> census_base;
    size_type staged_bytes{0};
    size_type mailbox_hits{0};
    size_type active_producers{0};
    size_type progress_version{0};
    size_type max_unresolved_events{0};
    size_type max_staged_bytes{0};
    size_type max_resident_events{0};
    size_type max_mailbox_hits{0};
    size_type event_admission_waits{0};
    size_type staged_bytes_waits{0};
    bool draining{false};
    bool stop_requested{false};
    bool stopped{false};
};

namespace
{
//---------------------------------------------------------------------------//
template<class C>
size_type checked_size(C const& container)
{
    CELER_VALIDATE(container.size() <= std::numeric_limits<size_type>::max(),
                   << "host container size " << container.size()
                   << " exceeds the configured Celeritas size type");
    return static_cast<size_type>(container.size());
}

//---------------------------------------------------------------------------//
template<class S>
void notify_state(S& state)
{
    ++state.progress_version;
    state.state_cv.notify_all();
}

//---------------------------------------------------------------------------//
template<class S>
void clear_ingress(S& state)
{
    for (auto& queue : state.ingress)
    {
        queue.clear();
    }
    state.staged_bytes = 0;
}

//---------------------------------------------------------------------------//
template<class S>
void set_terminal_error(S& state, std::exception_ptr error)
{
    if (!state.terminal_error)
    {
        state.terminal_error = std::move(error);
        state.stop_requested = true;
        clear_ingress(state);
        notify_state(state);
        state.work_cv.notify_all();
    }
}

//---------------------------------------------------------------------------//
template<class S>
void throw_if_failed(S const& state)
{
    if (state.terminal_error)
    {
        std::rethrow_exception(state.terminal_error);
    }
}

//---------------------------------------------------------------------------//
template<class S>
void validate_accepting(S const& state)
{
    throw_if_failed(state);
    CELER_VALIDATE(!state.draining && !state.stop_requested && !state.stopped,
                   << "optical transport service is no longer accepting "
                      "producer calls");
}

//---------------------------------------------------------------------------//
template<class S>
void cleanup_completed(S& state)
{
    for (auto iter = state.results.begin(); iter != state.results.end();)
    {
        if (state.events.is_complete(iter->first))
        {
            CELER_ASSERT(iter->second.hits.empty());
            CELER_ASSERT(!iter->second.pumping);
            auto& metrics = state.lane_metrics[*iter->second.lane];
            auto const elapsed = S::Clock::now() - iter->second.registered;
            metrics.retirement_time += elapsed;
            metrics.retirement_max = std::max(metrics.retirement_max, elapsed);
            ++metrics.retired_events;
            iter = state.results.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
    state.census_base.store(state.events.completion_watermark() + 1,
                            std::memory_order_relaxed);
}

//---------------------------------------------------------------------------//
template<class S>
bool has_pending_burst(S const& state, OpticalTransportService::LaneId lane)
{
    auto const& queue = state.ingress[*lane];
    return std::any_of(queue.begin(), queue.end(), [](auto const& command) {
        return command.type == OpticalTransportLaneCommandType::burst;
    });
}

//---------------------------------------------------------------------------//
template<class S>
void update_maxima(S& state)
{
    state.max_unresolved_events = std::max(state.max_unresolved_events,
                                           state.events.unresolved_count());
    state.max_staged_bytes
        = std::max(state.max_staged_bytes, state.staged_bytes);
    state.max_resident_events = std::max<size_type>(
        state.max_resident_events, checked_size(state.results));
    state.max_mailbox_hits
        = std::max(state.max_mailbox_hits, state.mailbox_hits);
}

//---------------------------------------------------------------------------//
template<class S>
void apply_progress(S& state,
                    OpticalTransportService::LaneId lane,
                    OpticalTransportLaneProgress progress)
{
    state.events.record_generation_progress(lane, progress.total_generated);

    for (auto& batch : progress.hit_batches)
    {
        auto result = state.results.find(batch.event);
        CELER_VALIDATE(result != state.results.end(),
                       << "lane " << lane
                       << " returned a hit for unregistered event "
                       << batch.event);
        CELER_VALIDATE(result->second.lane == lane,
                       << "lane " << lane << " returned a hit for event "
                       << batch.event << " assigned to lane "
                       << result->second.lane);
        state.mailbox_hits += batch.hits.size();
        auto& metrics = state.lane_metrics[*lane];
        metrics.mailbox_hits += batch.hits.size();
        metrics.hit_mail_high_water
            = std::max(metrics.hit_mail_high_water, metrics.mailbox_hits);
        result->second.hits.insert(result->second.hits.end(),
                                   std::make_move_iterator(batch.hits.begin()),
                                   std::make_move_iterator(batch.hits.end()));
    }

    // A report older than an already queued append is not a fresh census
    if (progress.census_fresh && !has_pending_burst(state, lane))
    {
        state.events.record_census(lane, progress.min_live_ordinal);
    }

    for (auto& [ordinal, result] : state.results)
    {
        if (!result.ready && state.events.is_transport_complete(ordinal))
        {
            result.ready = true;
        }
    }

    update_maxima(state);
    cleanup_completed(state);
    notify_state(state);
}

//---------------------------------------------------------------------------//
template<class S>
void advance_delivery(
    S& state, OpticalTransportService::LaneId lane, long ordinal)
{
    auto& delivery = state.delivery[*lane];
    delivery.completed.insert(ordinal);

    long delivered_through{-1};
    while (delivery.completed.erase(delivery.next_ordinal) != 0)
    {
        delivered_through = delivery.next_ordinal;
        delivery.next_ordinal += static_cast<long>(state.options.num_lanes);
    }
    if (delivered_through >= 0)
    {
        state.events.record_delivered_through(lane, delivered_through);
    }

    cleanup_completed(state);
    notify_state(state);
}

//---------------------------------------------------------------------------//
template<class S>
void log_lane_metrics(S const& state, OpticalTransportService::LaneId lane)
{
    auto const& metrics = state.lane_metrics[*lane];
    double const idle_seconds
        = std::chrono::duration<double>(metrics.idle_park_time).count();
    double const retirement_total
        = std::chrono::duration<double>(metrics.retirement_time).count();
    double const retirement_mean = metrics.retired_events > 0
                                       ? retirement_total
                                             / metrics.retired_events
                                       : 0;
    double const retirement_max
        = std::chrono::duration<double>(metrics.retirement_max).count();

    CELER_LOG_LOCAL(info)
        << "Optical streaming metrics: staged " << metrics.staged_bursts
        << " bursts with " << metrics.staged_photons << " photons; idle "
        << metrics.idle_parks << " parks for " << idle_seconds
        << " s; hit mail high-water " << metrics.hit_mail_high_water
        << " hits; pumped " << metrics.pump_hits_total << " hits over "
        << metrics.pump_count << " calls (max " << metrics.pump_hits_max
        << "); event retirement " << metrics.retired_events << " measured of "
        << metrics.registered_events << " registered, " << retirement_total
        << " s total (mean " << retirement_mean << " s, max " << retirement_max
        << " s)";
}

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
/*!
 * Construct a producer token.
 */
OpticalTransportService::ProducerToken::ProducerToken(
    std::shared_ptr<SharedState> state)
    : state_(std::move(state))
{
    CELER_EXPECT(state_);
}

//---------------------------------------------------------------------------//
OpticalTransportService::ProducerToken::ProducerToken(
    ProducerToken&& other) noexcept
    : state_(std::move(other.state_))
    , owned_events_(std::move(other.owned_events_))
    , open_events_(std::move(other.open_events_))
{
}

//---------------------------------------------------------------------------//
auto OpticalTransportService::ProducerToken::operator=(
    ProducerToken&& other) noexcept -> ProducerToken&
{
    if (this != &other)
    {
        this->release();
        state_ = std::move(other.state_);
        owned_events_ = std::move(other.owned_events_);
        open_events_ = std::move(other.open_events_);
    }
    return *this;
}

//---------------------------------------------------------------------------//
OpticalTransportService::ProducerToken::~ProducerToken()
{
    this->release();
}

//---------------------------------------------------------------------------//
/*!
 * Register an event and return its deterministic lane.
 */
auto OpticalTransportService::ProducerToken::register_event(long ordinal)
    -> LaneId
{
    OpticalTransportService::check_service(state_);
    CELER_VALIDATE(owned_events_.count(ordinal) == 0,
                   << "producer already owns optical event " << ordinal);

    owned_events_.insert(ordinal);
    try
    {
        open_events_.insert(ordinal);
        return OpticalTransportService::register_event(state_, ordinal);
    }
    catch (...)
    {
        open_events_.erase(ordinal);
        owned_events_.erase(ordinal);
        throw;
    }
}

//---------------------------------------------------------------------------//
/*!
 * Submit a bounded host-side burst.
 */
void OpticalTransportService::ProducerToken::submit_burst(
    long ordinal, size_type num_photons, size_type size_bytes)
{
    OpticalTransportService::check_service(state_);
    this->check_open(ordinal);
    OpticalTransportService::submit_burst(state_,
                                          {ordinal, num_photons, size_bytes});
}

//---------------------------------------------------------------------------//
/*!
 * Submit real optical generator records and charge their host storage.
 */
void OpticalTransportService::ProducerToken::submit_burst(
    long ordinal,
    std::vector<optical::GeneratorDistributionData> records,
    size_type num_photons)
{
    OpticalTransportService::check_service(state_);
    this->check_open(ordinal);

    OpticalTransportBurst burst;
    burst.event = ordinal;
    burst.num_photons = num_photons;
    burst.size_bytes = records.size()
                       * sizeof(optical::GeneratorDistributionData);
    burst.records = std::move(records);
    OpticalTransportService::submit_burst(state_, std::move(burst));
}

//---------------------------------------------------------------------------//
/*!
 * Explicitly close an event against future submissions.
 */
void OpticalTransportService::ProducerToken::close_event(long ordinal)
{
    OpticalTransportService::check_service(state_);
    this->check_open(ordinal);
    OpticalTransportService::close_event(state_, ordinal);
    open_events_.erase(ordinal);
}

//---------------------------------------------------------------------------//
/*!
 * Pump and wait until an owned, closed event is complete.
 */
void OpticalTransportService::ProducerToken::wait_until_complete(long ordinal)
{
    OpticalTransportService::check_service(state_);
    this->check_owned(ordinal);
    CELER_VALIDATE(open_events_.count(ordinal) == 0,
                   << "cannot wait for open optical event " << ordinal);
    OpticalTransportService::wait_until_complete(state_, ordinal);
    owned_events_.erase(ordinal);
}

//---------------------------------------------------------------------------//
/*!
 * Query an owned event's completion state.
 */
bool OpticalTransportService::ProducerToken::is_complete(long ordinal) const
{
    OpticalTransportService::check_service(state_);
    this->check_owned(ordinal);
    return OpticalTransportService::is_complete(state_, ordinal);
}

//---------------------------------------------------------------------------//
void OpticalTransportService::ProducerToken::check_owned(long ordinal) const
{
    CELER_VALIDATE(owned_events_.count(ordinal) != 0,
                   << "producer does not own optical event " << ordinal);
}

//---------------------------------------------------------------------------//
void OpticalTransportService::ProducerToken::check_open(long ordinal) const
{
    this->check_owned(ordinal);
    CELER_VALIDATE(open_events_.count(ordinal) != 0,
                   << "optical event " << ordinal << " is already closed");
}

//---------------------------------------------------------------------------//
void OpticalTransportService::ProducerToken::release() noexcept
{
    if (state_)
    {
        OpticalTransportService::release_producer(state_, open_events_);
        state_.reset();
        owned_events_.clear();
        open_events_.clear();
    }
}

//---------------------------------------------------------------------------//
/*!
 * Construct the standalone shell with one injected implementation per lane.
 */
OpticalTransportService::OpticalTransportService(
    Options options,
    LaneFactory make_lane,
    HitCallback hit_callback,
    ActionTimeCallback action_time_callback)
{
    CELER_VALIDATE(options.num_lanes > 0,
                   << "optical transport service requires at least one lane");
    CELER_VALIDATE(options.staged_bytes_limit > 0,
                   << "optical transport service requires a positive staged "
                      "byte limit");
    CELER_VALIDATE(
        options.base_ordinal >= 0,
        << "invalid negative optical base ordinal " << options.base_ordinal);
    CELER_VALIDATE(
        options.base_ordinal <= std::numeric_limits<long>::max()
                                    - static_cast<long>(options.num_lanes - 1),
        << "optical base ordinal " << options.base_ordinal
        << " is too large for " << options.num_lanes << " lanes");
    CELER_VALIDATE(make_lane, << "missing optical lane factory");

    state_ = std::make_shared<SharedState>(
        options, std::move(hit_callback), std::move(action_time_callback));
    lanes_.reserve(options.num_lanes);
    workers_.reserve(options.num_lanes);
    for (size_type i = 0; i < options.num_lanes; ++i)
    {
        LaneId const lane{i};
        auto impl = make_lane(lane);
        CELER_VALIDATE(impl,
                       << "invalid implementation for optical lane " << lane);
        lanes_.push_back(std::move(impl));
    }

    try
    {
        for (size_type i = 0; i < options.num_lanes; ++i)
        {
            workers_.emplace_back(
                [this, lane = LaneId{i}] { this->lane_loop(lane); });
        }
    }
    catch (...)
    {
        this->stop_and_join();
        throw;
    }
}

//---------------------------------------------------------------------------//
OpticalTransportService::~OpticalTransportService()
{
    std::lock_guard<std::mutex> lifecycle_lock{lifecycle_mutex_};
    this->stop_and_join();
}

//---------------------------------------------------------------------------//
/*!
 * Register one producer lifetime with the service.
 */
auto OpticalTransportService::make_producer() -> ProducerToken
{
    std::lock_guard<std::mutex> lock{state_->mutex};
    validate_accepting(*state_);
    ++state_->active_producers;
    notify_state(*state_);
    return ProducerToken{state_};
}

//---------------------------------------------------------------------------//
/*!
 * Pump currently available hits for one event on the calling thread.
 */
auto OpticalTransportService::pump(long ordinal) -> PumpResult
{
    return OpticalTransportService::pump(state_, ordinal, false);
}

//---------------------------------------------------------------------------//
/*!
 * Try to pump currently available hits without blocking on another pumper.
 */
auto OpticalTransportService::try_pump(long ordinal) -> PumpResult
{
    return OpticalTransportService::pump(state_, ordinal, true);
}

//---------------------------------------------------------------------------//
/*!
 * Query per-event completion.
 */
bool OpticalTransportService::is_complete(long ordinal) const
{
    return OpticalTransportService::is_complete(state_, ordinal);
}

//---------------------------------------------------------------------------//
/*!
 * Pump and wait until a registered, closed event is complete.
 */
void OpticalTransportService::wait_until_complete(long ordinal)
{
    OpticalTransportService::wait_until_complete(state_, ordinal);
}

//---------------------------------------------------------------------------//
/*!
 * Drain every closed event, stop all lanes, and join their threads.
 */
void OpticalTransportService::drain_and_stop()
{
    std::unique_lock<std::mutex> lifecycle_lock{lifecycle_mutex_};
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        if (state_->stopped)
        {
            throw_if_failed(*state_);
            return;
        }
        CELER_VALIDATE(state_->active_producers == 0,
                       << "cannot drain optical transport service with "
                       << state_->active_producers << " active producers");
        state_->draining = true;
        notify_state(*state_);
    }

    std::exception_ptr failure;
    try
    {
        while (true)
        {
            std::vector<long> ready;
            {
                std::unique_lock<std::mutex> lock{state_->mutex};
                throw_if_failed(*state_);
                if (state_->events.unresolved_count() == 0)
                {
                    break;
                }

                for (auto const& [ordinal, result] : state_->results)
                {
                    if (!result.delivered && !result.pumping
                        && (result.ready || !result.hits.empty()))
                    {
                        ready.push_back(ordinal);
                    }
                }

                if (ready.empty())
                {
                    size_type const version = state_->progress_version;
                    state_->state_cv.wait(lock, [&] {
                        return state_->terminal_error
                               || state_->events.unresolved_count() == 0
                               || state_->progress_version != version;
                    });
                    continue;
                }
            }

            for (long ordinal : ready)
            {
                OpticalTransportService::pump(state_, ordinal, false);
            }
        }
    }
    catch (...)
    {
        failure = std::current_exception();
    }

    this->stop_and_join();
    if (failure)
    {
        std::rethrow_exception(failure);
    }

    for (size_type i = 0; i < lanes_.size(); ++i)
    {
        lanes_[i]->finalize();
        if (state_->action_time_callback)
        {
            state_->action_time_callback(LaneId{i}, lanes_[i]->action_time());
        }
        if (state_->options.log_metrics)
        {
            log_lane_metrics(*state_, LaneId{i});
        }
    }
}

//---------------------------------------------------------------------------//
/*!
 * Snapshot bounded-memory and backpressure counters.
 */
auto OpticalTransportService::statistics() const -> Statistics
{
    std::lock_guard<std::mutex> lock{state_->mutex};
    Statistics result;
    result.unresolved_events = state_->events.unresolved_count();
    result.staged_bytes = state_->staged_bytes;
    result.resident_events = state_->results.size();
    result.mailbox_hits = state_->mailbox_hits;
    result.max_unresolved_events = state_->max_unresolved_events;
    result.max_staged_bytes = state_->max_staged_bytes;
    result.max_resident_events = state_->max_resident_events;
    result.max_mailbox_hits = state_->max_mailbox_hits;
    result.event_admission_waits = state_->event_admission_waits;
    result.staged_bytes_waits = state_->staged_bytes_waits;
    result.active_producers = state_->active_producers;
    result.completion_watermark = state_->events.completion_watermark();
    result.stopped = state_->stopped;
    return result;
}

//---------------------------------------------------------------------------//
void OpticalTransportService::check_service(
    std::shared_ptr<SharedState> const& state)
{
    CELER_VALIDATE(state, << "invalid optical producer token");
    std::lock_guard<std::mutex> lock{state->mutex};
    throw_if_failed(*state);
    CELER_VALIDATE(!state->stopped,
                   << "optical transport service has already stopped");
}

//---------------------------------------------------------------------------//
auto OpticalTransportService::register_event(
    std::shared_ptr<SharedState> const& state, long ordinal) -> LaneId
{
    std::unique_lock<std::mutex> lock{state->mutex};
    validate_accepting(*state);
    CELER_VALIDATE(ordinal >= 0,
                   << "invalid negative optical event ordinal " << ordinal);

    auto can_admit = [&] {
        long const first = state->events.completion_watermark() + 1;
        if (ordinal < first)
        {
            // Let the table diagnose duplicate or retired ordinals
            return state->events.admission_available();
        }
        size_type const offset = static_cast<size_type>(ordinal - first);
        return state->events.admission_available()
               && offset < state->events.unresolved_limit();
    };
    if (!can_admit())
    {
        ++state->event_admission_waits;
    }
    state->state_cv.wait(lock, [&] {
        return state->terminal_error || state->stop_requested || can_admit();
    });
    validate_accepting(*state);

    LaneId const lane = state->events.register_event(ordinal);
    SharedState::EventResult result;
    result.lane = lane;
    result.registered = SharedState::Clock::now();
    try
    {
        auto inserted = state->results.emplace(ordinal, std::move(result));
        CELER_ASSERT(inserted.second);
    }
    catch (...)
    {
        set_terminal_error(*state, std::current_exception());
        throw;
    }
    ++state->lane_metrics[*lane].registered_events;
    update_maxima(*state);
    notify_state(*state);
    return lane;
}

//---------------------------------------------------------------------------//
void OpticalTransportService::submit_burst(
    std::shared_ptr<SharedState> const& state, OpticalTransportBurst burst)
{
    std::unique_lock<std::mutex> lock{state->mutex};
    validate_accepting(*state);
    CELER_VALIDATE(burst.size_bytes > 0,
                   << "optical burst for event " << burst.event
                   << " has zero staged bytes");
    CELER_VALIDATE(burst.size_bytes <= state->options.staged_bytes_limit,
                   << "optical burst for event " << burst.event << " uses "
                   << burst.size_bytes << " staged bytes, exceeding limit "
                   << state->options.staged_bytes_limit);

    auto has_space = [&] {
        return state->staged_bytes
               <= state->options.staged_bytes_limit - burst.size_bytes;
    };
    if (!has_space())
    {
        ++state->staged_bytes_waits;
    }
    state->state_cv.wait(lock, [&] {
        return state->terminal_error || state->stop_requested || has_space();
    });
    validate_accepting(*state);

    state->events.submit_burst(burst.event, burst.num_photons);
    LaneId const lane{static_cast<size_type>(
        burst.event % static_cast<long>(state->options.num_lanes))};
    OpticalTransportLaneCommand command;
    command.type = OpticalTransportLaneCommandType::burst;
    size_type const size_bytes = burst.size_bytes;
    size_type const num_photons = burst.num_photons;
    command.burst = std::move(burst);
    try
    {
        state->ingress[*lane].push_back(std::move(command));
    }
    catch (...)
    {
        set_terminal_error(*state, std::current_exception());
        throw;
    }
    state->staged_bytes += size_bytes;
    auto& metrics = state->lane_metrics[*lane];
    ++metrics.staged_bursts;
    metrics.staged_photons += num_photons;
    update_maxima(*state);
    notify_state(*state);
    state->work_cv.notify_all();
}

//---------------------------------------------------------------------------//
void OpticalTransportService::close_event(
    std::shared_ptr<SharedState> const& state, long ordinal)
{
    std::lock_guard<std::mutex> lock{state->mutex};
    validate_accepting(*state);
    state->events.close_event(ordinal);
    LaneId const lane{static_cast<size_type>(
        ordinal % static_cast<long>(state->options.num_lanes))};
    OpticalTransportLaneCommand command;
    command.type = OpticalTransportLaneCommandType::close;
    command.burst.event = ordinal;
    try
    {
        state->ingress[*lane].push_back(std::move(command));
    }
    catch (...)
    {
        set_terminal_error(*state, std::current_exception());
        throw;
    }
    notify_state(*state);
    state->work_cv.notify_all();
}

//---------------------------------------------------------------------------//
auto OpticalTransportService::pump(
    std::shared_ptr<SharedState> const& state, long ordinal, bool try_lock)
    -> PumpResult
{
    std::unique_lock<std::mutex> pump_lock{state->pump_mutex, std::defer_lock};
    if (try_lock)
    {
        if (!pump_lock.try_lock())
        {
            return {false, false, true};
        }
    }
    else
    {
        pump_lock.lock();
    }

    std::vector<optical::DetectorHit> hits;
    LaneId lane;
    {
        std::lock_guard<std::mutex> lock{state->mutex};
        throw_if_failed(*state);
        if (state->events.is_complete(ordinal))
        {
            return {false, true, false};
        }

        auto result = state->results.find(ordinal);
        CELER_VALIDATE(result != state->results.end(),
                       << "optical event " << ordinal << " is not registered");
        lane = result->second.lane;
        auto& metrics = state->lane_metrics[*lane];
        ++metrics.pump_count;
        if (result->second.delivered
            || (result->second.hits.empty() && !result->second.ready))
        {
            return {false, false, false};
        }

        CELER_ASSERT(!result->second.pumping);
        result->second.pumping = true;
        hits.swap(result->second.hits);
        CELER_ASSERT(state->mailbox_hits >= hits.size());
        state->mailbox_hits -= hits.size();
        CELER_ASSERT(metrics.mailbox_hits >= hits.size());
        metrics.mailbox_hits -= hits.size();
        size_type const num_hits = checked_size(hits);
        metrics.pump_hits_total += num_hits;
        metrics.pump_hits_max
            = std::max<size_type>(metrics.pump_hits_max, num_hits);
        notify_state(*state);
    }

    try
    {
        if (state->hit_callback)
        {
            state->hit_callback(ordinal, hits);
        }
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock{state->mutex};
        auto result = state->results.find(ordinal);
        if (result != state->results.end())
        {
            result->second.pumping = false;
        }
        set_terminal_error(*state, std::current_exception());
        throw;
    }

    std::lock_guard<std::mutex> lock{state->mutex};
    throw_if_failed(*state);
    auto result = state->results.find(ordinal);
    CELER_ASSERT(result != state->results.end());
    result->second.pumping = false;
    if (result->second.ready && result->second.hits.empty())
    {
        result->second.delivered = true;
        advance_delivery(*state, lane, ordinal);
    }
    else
    {
        notify_state(*state);
    }
    return {true, state->events.is_complete(ordinal), false};
}

//---------------------------------------------------------------------------//
bool OpticalTransportService::is_complete(
    std::shared_ptr<SharedState> const& state, long ordinal)
{
    std::lock_guard<std::mutex> lock{state->mutex};
    throw_if_failed(*state);
    return state->events.is_complete(ordinal);
}

//---------------------------------------------------------------------------//
void OpticalTransportService::wait_until_complete(
    std::shared_ptr<SharedState> const& state, long ordinal)
{
    while (true)
    {
        if (OpticalTransportService::pump(state, ordinal, false).complete)
        {
            return;
        }

        std::vector<long> ready;
        std::unique_lock<std::mutex> lock{state->mutex};
        throw_if_failed(*state);
        if (state->events.is_complete(ordinal))
        {
            return;
        }
        auto const target = state->results.find(ordinal);
        CELER_ASSERT(target != state->results.end());
        if (!target->second.delivered && !target->second.pumping
            && (target->second.ready || !target->second.hits.empty()))
        {
            // Progress may arrive after the pump attempt but before this
            // lock. Retry instead of sleeping past that notification.
            lock.unlock();
            continue;
        }
        for (auto const& [ready_ordinal, result] : state->results)
        {
            if (ready_ordinal != ordinal && !result.delivered
                && !result.pumping && (result.ready || !result.hits.empty()))
            {
                ready.push_back(ready_ordinal);
            }
        }
        if (!ready.empty())
        {
            lock.unlock();
            for (long ready_ordinal : ready)
            {
                OpticalTransportService::pump(state, ready_ordinal, false);
            }
            continue;
        }
        size_type const version = state->progress_version;
        state->state_cv.wait(lock, [&] {
            return state->terminal_error || state->events.is_complete(ordinal)
                   || state->progress_version != version;
        });
    }
}

//---------------------------------------------------------------------------//
void OpticalTransportService::release_producer(
    std::shared_ptr<SharedState> const& state,
    std::unordered_set<long> const& open_events) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock{state->mutex};
        if (!open_events.empty() && !state->terminal_error && !state->stopped)
        {
            set_terminal_error(
                *state,
                std::make_exception_ptr(std::runtime_error(
                    "optical producer destroyed with an unclosed event")));
        }
        CELER_ASSERT(state->active_producers > 0);
        --state->active_producers;
        notify_state(*state);
    }
    catch (...)
    {
        // Producer destruction must not throw
    }
}

//---------------------------------------------------------------------------//
/*!
 * Run one lane on its single service-owned thread.
 */
void OpticalTransportService::lane_loop(LaneId lane)
{
    OpticalTransportLaneControl control;
    control.receive = [this, lane](auto& commands, bool block) {
        std::unique_lock<std::mutex> lock{state_->mutex};
        auto& queue = state_->ingress[*lane];
        auto& metrics = state_->lane_metrics[*lane];

        std::optional<SharedState::Clock::time_point> park_start;
        if (block && metrics.started && queue.empty()
            && !state_->stop_requested)
        {
            park_start = SharedState::Clock::now();
        }
        if (block)
        {
            state_->work_cv.wait(lock, [&] {
                return state_->stop_requested || !queue.empty();
            });
        }
        if (park_start)
        {
            ++metrics.idle_parks;
            metrics.idle_park_time += SharedState::Clock::now() - *park_start;
        }
        if (state_->stop_requested)
        {
            return OpticalTransportLaneControl::ReceiveStatus::stop;
        }
        if (queue.empty())
        {
            return OpticalTransportLaneControl::ReceiveStatus::idle;
        }

        size_type absorbed_bursts{0};
        commands.reserve(commands.size() + queue.size());
        while (!queue.empty())
        {
            auto command = std::move(queue.front());
            queue.pop_front();
            if (command.type == OpticalTransportLaneCommandType::burst)
            {
                CELER_ASSERT(state_->staged_bytes >= command.burst.size_bytes);
                state_->staged_bytes -= command.burst.size_bytes;
                ++absorbed_bursts;
            }
            commands.push_back(std::move(command));
        }
        if (absorbed_bursts > 0)
        {
            state_->events.record_bursts_absorbed(lane, absorbed_bursts);
        }
        metrics.started = true;
        notify_state(*state_);
        return OpticalTransportLaneControl::ReceiveStatus::work;
    };
    control.census_base = [state = state_] {
        return state->census_base.load(std::memory_order_relaxed);
    };
    control.publish = [this, lane](OpticalTransportLaneProgress progress) {
        std::lock_guard<std::mutex> lock{state_->mutex};
        if (!state_->terminal_error && !state_->stop_requested)
        {
            apply_progress(*state_, lane, std::move(progress));
        }
    };

    try
    {
        lanes_[*lane]->run(control);
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        set_terminal_error(*state_, std::current_exception());
    }
}

//---------------------------------------------------------------------------//
/*!
 * Stop and join every lane without throwing.
 */
void OpticalTransportService::stop_and_join() noexcept
{
    if (!state_)
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        state_->stop_requested = true;
        clear_ingress(*state_);
        notify_state(*state_);
        state_->work_cv.notify_all();
    }
    for (auto& worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        state_->stopped = true;
        notify_state(*state_);
    }
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
