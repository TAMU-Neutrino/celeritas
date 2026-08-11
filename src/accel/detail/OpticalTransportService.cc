//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalTransportService.cc
//---------------------------------------------------------------------------//
#include "OpticalTransportService.hh"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <exception>
#include <iterator>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "corecel/Assert.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
struct OpticalTransportService::SharedState
{
    enum class CommandType
    {
        burst,
        close
    };

    struct Command
    {
        CommandType type{CommandType::burst};
        OpticalTransportBurst burst;
    };

    struct EventResult
    {
        LaneId lane;
        std::vector<optical::DetectorHit> hits;
        bool ready{false};
        bool pumping{false};
        bool delivered{false};
    };

    struct LaneDelivery
    {
        long next_ordinal{-1};
        std::unordered_set<long> completed;
    };

    SharedState(Options const& opts, HitCallback callback)
        : options(opts)
        , events(opts.num_lanes, opts.unresolved_limit)
        , ingress(opts.num_lanes)
        , delivery(opts.num_lanes)
        , hit_callback(std::move(callback))
    {
        for (size_type i = 0; i < delivery.size(); ++i)
        {
            delivery[i].next_ordinal = static_cast<long>(i);
        }
    }

    Options options;
    mutable std::mutex mutex;
    std::mutex pump_mutex;
    std::condition_variable work_cv;
    std::condition_variable state_cv;
    OpticalEventTable events;
    std::vector<std::deque<Command>> ingress;
    std::vector<LaneDelivery> delivery;
    std::unordered_map<long, EventResult> results;
    HitCallback hit_callback;
    std::exception_ptr terminal_error;
    size_type staged_bytes{0};
    size_type mailbox_hits{0};
    size_type in_flight{0};
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
            iter = state.results.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
}

//---------------------------------------------------------------------------//
template<class S>
bool has_pending_burst(S const& state, OpticalTransportService::LaneId lane)
{
    auto const& queue = state.ingress[*lane];
    return std::any_of(queue.begin(), queue.end(), [](auto const& command) {
        return command.type == S::CommandType::burst;
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
    state.max_resident_events
        = std::max(state.max_resident_events, state.results.size());
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
        result->second.hits.insert(result->second.hits.end(),
                                   std::make_move_iterator(batch.hits.begin()),
                                   std::make_move_iterator(batch.hits.end()));
    }

    // A report older than an already queued append is not a fresh census
    if (progress.census_fresh && !has_pending_burst(state, lane))
    {
        state.events.record_census(lane, progress.min_live_ordinal);
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
    Options options, LaneFactory make_lane, HitCallback hit_callback)
{
    CELER_VALIDATE(options.num_lanes > 0,
                   << "optical transport service requires at least one lane");
    CELER_VALIDATE(options.staged_bytes_limit > 0,
                   << "optical transport service requires a positive staged "
                      "byte limit");
    CELER_VALIDATE(make_lane, << "missing optical lane factory");

    state_ = std::make_shared<SharedState>(options, std::move(hit_callback));
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
    return OpticalTransportService::pump(state_, ordinal);
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
                OpticalTransportService::pump(state_, ordinal);
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
    SharedState::Command command;
    command.type = SharedState::CommandType::burst;
    command.burst = burst;
    try
    {
        state->ingress[*lane].push_back(std::move(command));
    }
    catch (...)
    {
        set_terminal_error(*state, std::current_exception());
        throw;
    }
    state->staged_bytes += burst.size_bytes;
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
    SharedState::Command command;
    command.type = SharedState::CommandType::close;
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
auto OpticalTransportService::pump(std::shared_ptr<SharedState> const& state,
                                   long ordinal) -> PumpResult
{
    std::lock_guard<std::mutex> pump_lock{state->pump_mutex};

    std::vector<optical::DetectorHit> hits;
    LaneId lane;
    {
        std::lock_guard<std::mutex> lock{state->mutex};
        throw_if_failed(*state);
        if (state->events.is_complete(ordinal))
        {
            return {false, true};
        }

        auto result = state->results.find(ordinal);
        CELER_VALIDATE(result != state->results.end(),
                       << "optical event " << ordinal << " is not registered");
        if (result->second.delivered
            || (result->second.hits.empty() && !result->second.ready))
        {
            return {false, false};
        }

        CELER_ASSERT(!result->second.pumping);
        result->second.pumping = true;
        lane = result->second.lane;
        hits.swap(result->second.hits);
        CELER_ASSERT(state->mailbox_hits >= hits.size());
        state->mailbox_hits -= hits.size();
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
    return {true, state->events.is_complete(ordinal)};
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
        if (OpticalTransportService::pump(state, ordinal).complete)
        {
            return;
        }

        std::unique_lock<std::mutex> lock{state->mutex};
        throw_if_failed(*state);
        if (state->events.is_complete(ordinal))
        {
            return;
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
 * Process one lane's ingress queue on its single owner thread.
 */
void OpticalTransportService::lane_loop(LaneId lane)
{
    bool command_in_flight{false};
    try
    {
        while (true)
        {
            SharedState::Command command;
            {
                std::unique_lock<std::mutex> lock{state_->mutex};
                state_->work_cv.wait(lock, [&] {
                    return state_->stop_requested
                           || !state_->ingress[*lane].empty();
                });
                if (state_->stop_requested)
                {
                    return;
                }

                command = std::move(state_->ingress[*lane].front());
                state_->ingress[*lane].pop_front();
                ++state_->in_flight;
                command_in_flight = true;
                if (command.type == SharedState::CommandType::burst)
                {
                    CELER_ASSERT(
                        state_->staged_bytes >= command.burst.size_bytes);
                    state_->staged_bytes -= command.burst.size_bytes;
                    state_->events.record_bursts_absorbed(lane, 1);
                }
                notify_state(*state_);
            }

            OpticalTransportLaneProgress progress;
            if (command.type == SharedState::CommandType::burst)
            {
                progress = lanes_[*lane]->transport(command.burst);
            }
            else
            {
                progress = lanes_[*lane]->close_event(command.burst.event);
            }

            std::lock_guard<std::mutex> lock{state_->mutex};
            CELER_ASSERT(state_->in_flight > 0);
            --state_->in_flight;
            command_in_flight = false;
            if (state_->terminal_error || state_->stop_requested)
            {
                return;
            }
            apply_progress(*state_, lane, std::move(progress));
            if (command.type == SharedState::CommandType::close)
            {
                auto result = state_->results.find(command.burst.event);
                CELER_VALIDATE(result != state_->results.end(),
                               << "closed unregistered optical event "
                               << command.burst.event);
                CELER_VALIDATE(!result->second.ready,
                               << "optical event " << command.burst.event
                               << " was closed more than once");
                result->second.ready = true;
                notify_state(*state_);
            }
        }
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        if (command_in_flight)
        {
            CELER_ASSERT(state_->in_flight > 0);
            --state_->in_flight;
        }
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
