//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalLane.cc
//---------------------------------------------------------------------------//
#include "OpticalLane.hh"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "corecel/io/Logger.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/ScopedProfiling.hh"
#include "celeritas/optical/CoreParams.hh"
#include "celeritas/optical/CoreState.hh"
#include "celeritas/optical/SimParams.hh"
#include "celeritas/optical/Transporter.hh"
#include "celeritas/optical/gen/GeneratorAction.hh"
#include "celeritas/phys/GeneratorRegistry.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Host-side instrumentation for streaming optical transport.
 *
 * This state exists before the consumer starts so event registration can be
 * timestamped in InitializeEvent. Each field is updated by either the producer
 * or consumer alone; pump fields are updated under the existing stream mutex.
 */
struct OpticalLane::Instrumentation
{
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;

    struct EventRegistration
    {
        long ordinal;
        Clock::time_point time;
    };

    struct Snapshot
    {
        size_type staged_bursts{0};
        size_type staged_photons{0};
        size_type idle_parks{0};
        double idle_park_seconds{0};
        size_type hit_mail_high_water{0};
        size_type pump_count{0};
        size_type pump_hits_total{0};
        size_type pump_hits_max{0};
        size_type registered_events{0};
        size_type retired_events{0};
        double retirement_seconds_total{0};
        double retirement_seconds_mean{0};
        double retirement_seconds_max{0};
    };

    void register_event(long ordinal)
    {
        pending_events.push_back({ordinal, Clock::now()});
        ++registered_events;
    }

    void record_stage(size_type photons)
    {
        ++staged_bursts;
        staged_photons += photons;
    }

    std::deque<EventRegistration> take_registrations()
    {
        std::deque<EventRegistration> result;
        result.swap(pending_events);
        return result;
    }

    void observe_events(std::deque<EventRegistration> const& registrations)
    {
        for (auto const& event : registrations)
        {
            events.push_back(event);
        }
    }

    void record_park(Duration elapsed)
    {
        ++idle_parks;
        idle_park_time += elapsed;
    }

    void record_pump(size_type hits)
    {
        ++pump_count;
        pump_hits_total += hits;
        pump_hits_max = std::max(pump_hits_max, hits);
        hit_mail_high_water = std::max(hit_mail_high_water, hits);
    }

    void retire_through(long ordinal)
    {
        auto const now = Clock::now();
        while (!events.empty() && events.front().ordinal <= ordinal)
        {
            Duration const elapsed = now - events.front().time;
            retirement_time += elapsed;
            retirement_max = std::max(retirement_max, elapsed);
            ++retired_events;
            events.pop_front();
        }
    }

    Snapshot snapshot() const
    {
        Snapshot result;
        result.staged_bursts = staged_bursts;
        result.staged_photons = staged_photons;
        result.idle_parks = idle_parks;
        result.idle_park_seconds
            = std::chrono::duration<double>(idle_park_time).count();
        result.hit_mail_high_water = hit_mail_high_water;
        result.pump_count = pump_count;
        result.pump_hits_total = pump_hits_total;
        result.pump_hits_max = pump_hits_max;
        result.registered_events = registered_events;
        result.retired_events = retired_events;
        result.retirement_seconds_total
            = std::chrono::duration<double>(retirement_time).count();
        result.retirement_seconds_mean = retired_events > 0
                                             ? result.retirement_seconds_total
                                                   / retired_events
                                             : 0;
        result.retirement_seconds_max
            = std::chrono::duration<double>(retirement_max).count();
        return result;
    }

    std::deque<EventRegistration> pending_events;
    std::deque<EventRegistration> events;
    size_type staged_bursts{0};
    size_type staged_photons{0};
    size_type idle_parks{0};
    Duration idle_park_time{Duration::zero()};
    size_type hit_mail_high_water{0};
    size_type pump_count{0};
    size_type pump_hits_total{0};
    size_type pump_hits_max{0};
    size_type registered_events{0};
    size_type retired_events{0};
    Duration retirement_time{Duration::zero()};
    Duration retirement_max{Duration::zero()};
};

//---------------------------------------------------------------------------//
/*!
 * Producer-consumer channel for streaming injection.
 *
 * The producer (Geant4 worker) appends record bursts and collects hits; the
 * consumer owns the optical state and runs the transport loop. All fields
 * are guarded by \c mutex except the thread handle.
 */
struct OpticalLane::Streaming
{
    struct Burst
    {
        long event{0};
        size_type photons{0};
        std::deque<Instrumentation::EventRegistration> registrations;
        std::vector<DistributionData> records;
    };

    std::mutex mutex;
    std::condition_variable cv;  //!< wakes the consumer (burst/stop)
    std::condition_variable cv_host;  //!< wakes barrier waits (drained)

    std::vector<Burst> staged;  //!< FIFO of bursts not yet absorbed
    std::vector<optical::DetectorHit> hit_mail;  //!< hits awaiting delivery
    long drained_event{-1};  //!< all events through this ordinal are done
    bool idle{true};  //!< consumer parked with nothing in flight
    bool stop{false};  //!< request consumer exit at next drain

    std::thread worker;

    // Safety net for destruction without Finalize: join before the members
    // the consumer references are torn down
    ~Streaming()
    {
        if (worker.joinable())
        {
            {
                std::lock_guard<std::mutex> lock{mutex};
                stop = true;
            }
            cv.notify_one();
            worker.join();
        }
    }
};

//---------------------------------------------------------------------------//
/*!
 * Construct with shared transport data and lane-local state.
 */
OpticalLane::OpticalLane(
    std::shared_ptr<optical::Transporter> transport,
    std::shared_ptr<optical::GeneratorAction const> generate,
    StreamId stream_id,
    size_type num_track_slots,
    HitCallbackFunc user_hit_callback,
    bool streaming)
    : transport_(std::move(transport))
    , generate_(std::move(generate))
    , user_hit_callback_(std::move(user_hit_callback))
{
    CELER_EXPECT(transport_);
    CELER_EXPECT(generate_);

    auto const& optical_params = *transport_->params();
    auto memspace = celeritas::device() ? MemSpace::device : MemSpace::host;
    if (memspace == MemSpace::device)
    {
        state_ = std::make_shared<optical::CoreState<MemSpace::device>>(
            optical_params, stream_id, num_track_slots);
    }
    else
    {
        state_ = std::make_shared<optical::CoreState<MemSpace::host>>(
            optical_params, stream_id, num_track_slots);
    }

    // Allocate auxiliary data
    if (optical_params.aux_reg())
    {
        state_->aux() = std::make_shared<AuxStateVec>(
            *optical_params.aux_reg(), memspace, stream_id, num_track_slots);
    }

    if (streaming)
    {
        metrics_ = std::make_shared<Instrumentation>();
    }
}

OpticalLane::~OpticalLane() = default;

//---------------------------------------------------------------------------//
/*!
 * Get the accumulated action times.
 */
auto OpticalLane::GetActionTime() const -> MapStrDbl
{
    CELER_EXPECT(*this);
    return transport_->get_action_times(*state_->aux());
}

//---------------------------------------------------------------------------//
/*!
 * Record the start of an event for host-side instrumentation.
 */
void OpticalLane::RegisterEvent(long event_ordinal)
{
    CELER_EXPECT(metrics_);
    metrics_->register_event(event_ordinal);
}

//---------------------------------------------------------------------------//
/*!
 * Hand the buffered records to the consumer thread and return immediately.
 */
void OpticalLane::StageStreaming(std::vector<DistributionData>& buffer,
                                 size_type& num_photons,
                                 long event_ordinal)
{
    CELER_EXPECT(*this);

    if (!stream_)
    {
        this->StartConsumer();
    }
    if (buffer.empty())
    {
        return;
    }

    ScopedProfiling profile_this{"stage"};

    if (celeritas::device())
    {
        CELER_LOG_LOCAL(debug)
            << "Staging " << num_photons << " optical photons of event "
            << event_ordinal << " for streaming transport";
    }

    auto& sx = *stream_;
    size_type const staged_photons = num_photons;
    {
        std::lock_guard<std::mutex> lock{sx.mutex};
        Streaming::Burst burst;
        burst.event = event_ordinal < 0 ? 0 : event_ordinal;
        burst.photons = num_photons;
        burst.registrations = metrics_->take_registrations();
        burst.records = std::move(buffer);
        sx.staged.push_back(std::move(burst));
    }
    sx.cv.notify_one();
    metrics_->record_stage(staged_photons);

    buffer.clear();
    num_photons = 0;
}

//---------------------------------------------------------------------------//
/*!
 * Deliver collected hits on the calling thread; return delivered-through.
 */
long OpticalLane::PumpStreaming()
{
    std::lock_guard<std::mutex> pump_lock{pump_mutex_};
    return this->PumpStreamingImpl();
}

//---------------------------------------------------------------------------//
/*!
 * Try to deliver collected hits without waiting for another pumper.
 */
long OpticalLane::TryPumpStreaming()
{
    std::unique_lock<std::mutex> pump_lock{pump_mutex_, std::try_to_lock};
    if (!pump_lock)
    {
        return -1;
    }
    return this->PumpStreamingImpl();
}

//---------------------------------------------------------------------------//
/*!
 * Deliver collected hits while holding the exclusive pump gate.
 */
long OpticalLane::PumpStreamingImpl()
{
    if (!stream_)
    {
        return -1;
    }

    std::vector<optical::DetectorHit> hits;
    long cursor;
    {
        std::lock_guard<std::mutex> lock{stream_->mutex};
        hits.swap(stream_->hit_mail);
        cursor = stream_->drained_event;
        metrics_->record_pump(hits.size());
    }
    if (!hits.empty() && user_hit_callback_)
    {
        user_hit_callback_(make_span(hits));
    }
    if (cursor > delivered_through_)
    {
        delivered_through_ = cursor;
    }
    return delivered_through_;
}

//---------------------------------------------------------------------------//
/*!
 * Start the consumer thread (first stage on this stream).
 */
void OpticalLane::StartConsumer()
{
    CELER_EXPECT(!stream_);

#if CELERITAS_CORE_GEO == CELERITAS_CORE_GEO_GEANT4
    CELER_VALIDATE(celeritas::device(),
                   << "streaming optical transport requires a device or a "
                      "non-Geant4 optical geometry: host navigation with "
                      "the Geant4 backend uses per-thread geometry state "
                      "(split classes, touchable allocators) that belongs "
                      "to the worker thread and cannot be used from the "
                      "consumer thread");
#endif

    stream_ = std::make_shared<Streaming>();
    stream_->worker = std::thread([this] { this->ConsumerLoop(); });
    // Status level so the engagement marker survives default verbosity
    CELER_LOG_LOCAL(status) << "Started streaming optical transport consumer";
}

//---------------------------------------------------------------------------//
/*!
 * Request consumer exit and join it.
 */
void OpticalLane::StopConsumer()
{
    if (!stream_ || !stream_->worker.joinable())
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock{stream_->mutex};
        stream_->stop = true;
    }
    stream_->cv.notify_one();
    stream_->worker.join();
}

//---------------------------------------------------------------------------//
/*!
 * Block until everything staged has been transported to completion.
 */
void OpticalLane::WaitStreamingDrained()
{
    if (!stream_)
    {
        return;
    }
    auto& sx = *stream_;
    std::unique_lock<std::mutex> lock{sx.mutex};
    sx.cv_host.wait(lock, [&sx] { return sx.idle && sx.staged.empty(); });
}

//---------------------------------------------------------------------------//
/*!
 * Log transport and streaming instrumentation at teardown.
 */
void OpticalLane::LogFinalization() const
{
    auto const& accum = state_->accum();
    CELER_ASSERT(state_->aux());
    auto const& gen = generate_->counters(*state_->aux());
    CELER_LOG_LOCAL(info)
        << "Finalizing Celeritas after " << accum.steps
        << " optical steps (over " << accum.step_iters << " step iterations)"
        << " from " << gen.accum.num_generated
        << " optical photons generated from " << gen.accum.buffer_size
        << " distributions"
        << "; absorbed " << absorb_bursts_ << " bursts over " << absorb_drains_
        << " drains (max " << absorb_max_ << ")";

    if (metrics_)
    {
        auto const metrics = metrics_->snapshot();
        CELER_LOG_LOCAL(info)
            << "Optical streaming metrics: staged " << metrics.staged_bursts
            << " bursts with " << metrics.staged_photons << " photons; idle "
            << metrics.idle_parks << " parks for " << metrics.idle_park_seconds
            << " s; hit mail high-water " << metrics.hit_mail_high_water
            << " hits; pumped " << metrics.pump_hits_total << " hits over "
            << metrics.pump_count << " calls (max " << metrics.pump_hits_max
            << "); event retirement " << metrics.retired_events
            << " measured of " << metrics.registered_events << " registered, "
            << metrics.retirement_seconds_total << " s total (mean "
            << metrics.retirement_seconds_mean << " s, max "
            << metrics.retirement_seconds_max << " s)";
    }

    if (!gen.counters.empty())
    {
        CELER_LOG_LOCAL(warning)
            << "Not all optical photons were tracked at the end of the "
               "stepping loop: "
            << gen.counters.num_pending << " queued photons from "
            << gen.counters.buffer_size << " distributions";
    }
}

//---------------------------------------------------------------------------//
/*!
 * Transport one service burst synchronously on the owning lane thread.
 *
 * This adapter is separate from the existing facade's streaming methods. It
 * installs a temporary hit sink so detector hits return through the service
 * mailbox, then uses the established blocking insert-and-transport path.
 */
auto OpticalLane::transport(OpticalTransportBurst const& burst)
    -> OpticalTransportLaneProgress
{
    CELER_EXPECT(*this);
    CELER_VALIDATE(!stream_,
                   << "optical lane cannot mix service and facade streaming");
    CELER_VALIDATE(!burst.records.empty(),
                   << "service burst for event " << burst.event
                   << " has no optical distributions");

    size_type expected_photons{0};
    for (auto const& record : burst.records)
    {
        expected_photons += record.num_photons;
    }
    CELER_VALIDATE(expected_photons == burst.num_photons,
                   << "service burst for event " << burst.event << " has "
                   << burst.num_photons << " photons but its distributions "
                   << "contain " << expected_photons);

    ++absorb_drains_;
    ++absorb_bursts_;
    absorb_max_ = 1;

    OpticalTransportLaneProgress result;
    OpticalTransportHitBatch batch;
    batch.event = burst.event;
    state_->hit_sink([&batch](Span<optical::DetectorHit const> hits) {
        batch.hits.insert(batch.hits.end(), hits.begin(), hits.end());
    });

    try
    {
        generate_->insert(*state_, make_span(burst.records));
        auto counters = state_->sync_get_counters();
        counters.num_pending += burst.num_photons;
        state_->sync_put_counters(counters);
        (*transport_)(*state_);
    }
    catch (...)
    {
        state_->hit_sink(nullptr);
        throw;
    }
    state_->hit_sink(nullptr);

    result.total_generated
        = generate_->counters(*state_->aux()).accum.num_generated;
    if (!batch.hits.empty())
    {
        result.hit_batches.push_back(std::move(batch));
    }
    result.census_fresh = true;
    return result;
}

//---------------------------------------------------------------------------//
/*!
 * Report completion progress for an explicitly closed service event.
 */
auto OpticalLane::close_event(long) -> OpticalTransportLaneProgress
{
    CELER_EXPECT(*this);
    CELER_VALIDATE(!stream_,
                   << "optical lane cannot mix service and facade streaming");

    OpticalTransportLaneProgress result;
    result.total_generated
        = generate_->counters(*state_->aux()).accum.num_generated;
    result.census_fresh = true;
    return result;
}

//---------------------------------------------------------------------------//
/*!
 * Emit lane-local transport finalization data.
 */
void OpticalLane::finalize()
{
    this->LogFinalization();
}

//---------------------------------------------------------------------------//
/*!
 * Consumer thread: a persistent transport loop fed by staged bursts.
 *
 * The loop absorbs staged bursts between step iterations, so the drain-out
 * tail of one event's photons transports the next events' instead of
 * idling. When nothing is staged, pending, or alive, the consumer publishes
 * the drain cursor and parks on the condition variable.
 */
void OpticalLane::ConsumerLoop()
{
    // Make the process device current on this thread, as worker threads do
    activate_device_local();

    auto& sx = *stream_;
    auto& state = *state_;

    // Collect hits under the mailbox lock; the producer delivers them
    state.hit_sink([&sx](Span<optical::DetectorHit const> hits) {
        std::lock_guard<std::mutex> lock{sx.mutex};
        sx.hit_mail.insert(sx.hit_mail.end(), hits.begin(), hits.end());
    });

    size_type const max_stall = transport_->params()->sim()->max_step_iters();

    size_type iter{0};
    size_type stall_iters{0};
    size_type last_inflight{0};
    size_type last_cut{0};
    size_type last_errored{0};
    long absorbed_event{-1};

    auto counters = state.sync_get_counters();

    while (true)
    {
        // Absorb every burst staged so far. Taking work must clear the
        // idle flag in the same critical section: a barrier that observed
        // (idle && staged.empty()) between the swap and the transport
        // would otherwise return while photons are in flight.
        std::vector<Streaming::Burst> bursts;
        {
            std::lock_guard<std::mutex> lock{sx.mutex};
            bursts.swap(sx.staged);
            if (!bursts.empty())
            {
                sx.idle = false;
            }
        }
        if (!bursts.empty())
        {
            ++absorb_drains_;
            absorb_bursts_ += bursts.size();
            if (bursts.size() > absorb_max_)
            {
                absorb_max_ = bursts.size();
            }
            for (auto& b : bursts)
            {
                metrics_->observe_events(b.registrations);
                generate_->append(state, make_span(b.records));
                absorbed_event = b.event;
                // Cumulative photon total at the end of this event, against
                // which the generator's progress says when the event's
                // photons have all been created
                staged_photons_ += b.photons;
                staged_.push_back({b.event, staged_photons_});
            }
            counters = state.sync_get_counters();
            stall_iters = 0;
        }

        if (has_transportable_work(counters))
        {
            counters = transport_->step_once(state, iter++, census_base_);

            // How far the generator has got: everything up to here has been
            // turned into tracks, so an event below it can only still be
            // represented by a live track or a pending re-emission record --
            // both of which the census sees.
            if (!staged_.empty())
            {
                size_type const made
                    = generate_->counters(*state.aux()).accum.num_generated;
                while (!staged_.empty() && staged_.front().second <= made)
                {
                    fully_generated_ = staged_.front().first;
                    staged_.pop_front();
                }
            }

            // Retire events the census says hold no photons anywhere. This
            // is what makes the cursor advance DURING transport: without it
            // the only completion signal is the loop going empty, which
            // under continuous injection may never happen, so nothing could
            // be released until the end of the run.
            //
            // Only THIS iteration's census may be acted on, and only when no
            // distribution was written during it. A record stored mid-census
            // by a parent that died in the same iteration is in neither the
            // generator's fold (which ran before the record existed) nor the
            // live-track close (the parent is dead). And between censuses the
            // fully-generated cursor keeps advancing, so a stale minimum
            // could licence retiring an event whose tracks -- and their
            // re-emission records -- only came into existence after that
            // census closed. Waiting costs at most one census period.
            bool const census_fresh
                = transport_->census_enabled()
                  && ((iter - 1) % transport_->census_period() == 0)
                  && counters.num_dist_written == 0;
            if (census_fresh
                && counters.min_live_event_rel < optical::event_ring)
            {
                long const oldest_live
                    = census_base_
                      + static_cast<long>(counters.min_live_event_rel);
                // Every event before the oldest live one is finished, but
                // only up to what has actually been staged and generated:
                // an event whose photons have not all been created yet has
                // nothing live to find.
                long const done = std::min(oldest_live - 1, fully_generated_);
                if (done > published_event_)
                {
                    published_event_ = done;
                    {
                        std::lock_guard<std::mutex> lock{sx.mutex};
                        if (done > sx.drained_event)
                        {
                            sx.drained_event = done;
                        }
                    }
                    metrics_->retire_through(done);
                }
                // Keep the reduction's reference within a ring of everything
                // in flight
                census_base_ = published_event_ + 1;
            }

            // Mirror the per-flush statistics of the blocking loop
            state.accum().steps += counters.num_active;
            ++state.accum().step_iters;
            state.accum().num_cut += counters.num_cut - last_cut;
            state.accum().num_errored += counters.num_errored - last_errored;
            last_cut = counters.num_cut;
            last_errored = counters.num_errored;

            // The no-progress breaker replaces the per-flush iteration cap:
            // any change of the in-flight population (deaths, generations,
            // or injected work) counts as progress
            size_type inflight = counters.num_pending + counters.num_alive;
            if (inflight != last_inflight)
            {
                stall_iters = 0;
            }
            last_inflight = inflight;

            if (CELER_UNLIKELY(++stall_iters >= max_stall))
            {
                CELER_LOG_LOCAL(error)
                    << "Streaming optical transport made no progress over "
                    << max_stall << " step iterations: aborting "
                    << counters.num_alive << " alive tracks and "
                    << counters.num_pending << " queued photons";
                state.accum().num_cut += counters.num_active
                                         + counters.num_pending;
                transport_->params()->gen_reg()->reset(*state.aux());
                state.reset();
                counters = state.sync_get_counters();
                stall_iters = 0;
                last_inflight = 0;
                last_cut = 0;
                last_errored = 0;
            }
            continue;
        }

        // Nothing in flight: publish the drain cursor and park
        {
            std::unique_lock<std::mutex> lock{sx.mutex};
            if (!sx.staged.empty())
            {
                // More work arrived while stepping
                continue;
            }
            sx.drained_event = absorbed_event;
            metrics_->retire_through(absorbed_event);
            sx.idle = true;
            ++state.accum().flushes;
            sx.cv_host.notify_all();
            if (sx.stop)
            {
                break;
            }
            auto const park_start = Instrumentation::Clock::now();
            sx.cv.wait(lock, [&sx] { return !sx.staged.empty() || sx.stop; });
            metrics_->record_park(Instrumentation::Clock::now() - park_start);
            sx.idle = false;
        }
    }

    state.hit_sink(nullptr);
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
