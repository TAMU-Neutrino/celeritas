//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/LocalOpticalGenOffload.cc
//---------------------------------------------------------------------------//
#include "LocalOpticalGenOffload.hh"

#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <G4EventManager.hh>
#include <G4MTRunManager.hh>

#include "corecel/io/Logger.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/ScopedProfiling.hh"
#include "geocel/GeantUtils.hh"
#include "celeritas/optical/CoreParams.hh"
#include "celeritas/optical/CoreState.hh"
#include "celeritas/optical/SimParams.hh"
#include "celeritas/optical/Transporter.hh"
#include "celeritas/optical/gen/GeneratorAction.hh"
#include "celeritas/phys/GeneratorRegistry.hh"

#include "SetupOptions.hh"
#include "SharedParams.hh"

namespace celeritas
{
//---------------------------------------------------------------------------//
/*!
 * Producer-consumer channel for streaming injection.
 *
 * The producer (Geant4 worker) appends record bursts and collects hits; the
 * consumer owns the optical state and runs the transport loop. All fields
 * are guarded by \c mutex except the thread handle.
 */
struct LocalOpticalGenOffload::Streaming
{
    struct Burst
    {
        long event{0};
        size_type photons{0};
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
    // the consumer references are torn down (this struct is declared after
    // them, so it is destroyed first)
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
 * Construct with options and shared data.
 */
LocalOpticalGenOffload::LocalOpticalGenOffload(SetupOptions const& options,
                                               SharedParams& params)
{
    CELER_VALIDATE(params.mode() == SharedParams::Mode::enabled,
                   << "cannot create local optical offload when Celeritas "
                      "offloading is disabled");

    if (options.optical)
    {
        streaming_ = options.optical->streaming.enabled;
#if CELERITAS_CORE_GEO == CELERITAS_CORE_GEO_GEANT4
        if (streaming_ && !celeritas::device())
        {
            // Host transport with the Geant4 navigator cannot run on the
            // consumer thread: its split-class data and touchable
            // allocators belong to the worker. Fall back to the blocking
            // flush rather than failing -- streaming is the default, and a
            // default has to work on every machine, just not equally fast.
            CELER_LOG_LOCAL(warning)
                << "Streaming optical transport is unavailable on the host "
                   "with the Geant4 geometry: falling back to a synchronous "
                   "flush. Build with VecGeom or run on a device for the "
                   "faster path.";
            streaming_ = false;
        }
#endif
        if (streaming_)
        {
            // Keep the user hit callback for producer-side delivery: in
            // streaming mode the detector action routes hits to the
            // consumer's sink instead of calling it on the transport thread
            user_hit_callback_ = options.optical->detectors.callback;
        }
    }

    // Save a pointer to the optical transporter
    transport_ = params.optical_problem_loaded().transporter;
    CELER_ASSERT(transport_);

    CELER_ASSERT(transport_->params());
    auto const& optical_params = *transport_->params();

    // Check the thread ID and MT model
    validate_geant_threading(optical_params.sizes().streams);

    // Save a pointer to the generator action
    generate_ = std::dynamic_pointer_cast<optical::GeneratorAction const>(
        params.optical_problem_loaded().generator);
    CELER_VALIDATE(generate_, << "invalid optical GeneratorAction");

    // Number of optical photons to buffer before offloading
    auto const& sizes = optical_params.sizes();
    auto_flush_ = sizes.primaries;

    auto stream_id = id_cast<StreamId>(get_geant_thread_id());

    // Allocate thread-local state data
    auto memspace = celeritas::device() ? MemSpace::device : MemSpace::host;
    if (memspace == MemSpace::device)
    {
        state_ = std::make_shared<optical::CoreState<MemSpace::device>>(
            optical_params, stream_id, sizes.tracks);
    }
    else
    {
        state_ = std::make_shared<optical::CoreState<MemSpace::host>>(
            optical_params, stream_id, sizes.tracks);
    }

    // Allocate auxiliary data
    if (optical_params.aux_reg())
    {
        state_->aux() = std::make_shared<AuxStateVec>(
            *optical_params.aux_reg(), memspace, stream_id, sizes.tracks);
    }

    CELER_ENSURE(*this);
}

//---------------------------------------------------------------------------//
/*!
 * Initialize with options and shared data.
 */
void LocalOpticalGenOffload::Initialize(SetupOptions const& options,
                                        SharedParams& params)
{
    *this = LocalOpticalGenOffload(options, params);
}

//---------------------------------------------------------------------------//
/*!
 * Set the event ID and reseed the Celeritas RNG at the start of an event.
 */
void LocalOpticalGenOffload::InitializeEvent(int id)
{
    CELER_EXPECT(*this);
    CELER_EXPECT(id >= 0);

    event_id_ = id_cast<UniqueEventId>(id);

    if (this->StreamingEnabled())
    {
        // The caller supplies a monotonic event ordinal (under Geant4 MT
        // each worker sees an increasing subset of a global sequence); it
        // tags this event's staged bursts so the drain cursor is in the
        // caller's coordinates
        CELER_VALIDATE(id >= event_ordinal_,
                       << "streaming event ordinal went backwards (" << id
                       << " after " << event_ordinal_ << ")");
        bool const first = event_ordinal_ < 0;
        event_ordinal_ = id;
        if (!first)
        {
            // Tracks from earlier events may still be in flight on the
            // consumer: reseeding the slot RNGs mid-transport would corrupt
            // their streams, so streaming seeds once, from the first event
            if (CELER_UNLIKELY(!reseed_note_logged_))
            {
                reseed_note_logged_ = true;
                CELER_LOG_LOCAL(status)
                    << "Streaming optical transport: per-event RNG "
                       "reseeding is disabled after the first event";
            }
            return;
        }
    }

    if constexpr (CELERITAS_RESEED == CELERITAS_RESEED_TRACKSLOT)
    {
        if (!(G4Threading::IsMultithreadedApplication()
              && G4MTRunManager::SeedOncePerCommunication()))
        {
            // Since Geant4 schedules events dynamically, reseed the Celeritas
            // RNGs using the Geant4 event ID for reproducibility. This
            // guarantees that an event can be reproduced given the event ID.
            state_->reseed(transport_->params()->rng(),
                           id_cast<UniqueEventId>(id));
        }
    }
}

//---------------------------------------------------------------------------//
/*!
 * Buffer distribution data for generating optical photons.
 */
void LocalOpticalGenOffload::Push(
    optical::GeneratorDistributionData const& data)
{
    CELER_EXPECT(*this);
    CELER_EXPECT(data);

    ScopedProfiling profile_this{"push"};

    buffer_.push_back(data);
    num_photons_ += data.num_photons;

    if (num_photons_ >= auto_flush_)
    {
        if (this->StreamingEnabled())
        {
            // Watermark pressure: hand the records over without blocking
            this->StageStreaming();
        }
        else
        {
            this->Flush();
        }
    }
}

//---------------------------------------------------------------------------//
/*!
 * Generate and transport optical photons from the buffered distribution data.
 */
void LocalOpticalGenOffload::Flush()
{
    CELER_EXPECT(*this);

    if (this->StreamingEnabled())
    {
        // Barrier with unchanged semantics: everything staged (including
        // the tail of the buffer) is transported and every hit is
        // delivered on this thread before returning
        ScopedProfiling profile_this("flush-streaming");
        this->StageStreaming();
        this->WaitStreamingDrained();
        this->PumpStreaming();
        return;
    }

    if (buffer_.empty())
    {
        return;
    }

    ScopedProfiling profile_this("flush");

    //! \todo Duplicated in \c LocalTransporter
    if (event_manager_ || !event_id_)
    {
        if (CELER_UNLIKELY(!event_manager_))
        {
            // Save the event manager pointer, thereby marking that
            // *subsequent* events need to have their IDs checked as well
            event_manager_ = G4EventManager::GetEventManager();
            CELER_ASSERT(event_manager_);
        }

        G4Event const* event = event_manager_->GetConstCurrentEvent();
        CELER_ASSERT(event);
        if (event_id_ != id_cast<UniqueEventId>(event->GetEventID()))
        {
            // The event ID has changed: reseed it
            this->InitializeEvent(event->GetEventID());
        }
    }
    CELER_ASSERT(event_id_);

    if (celeritas::device())
    {
        CELER_LOG_LOCAL(debug)
            << "Transporting " << num_photons_
            << " optical photons from event " << event_id_.unchecked_get()
            << " with Celeritas";
    }

    // Copy the buffered distributions to device
    generate_->insert(*state_, make_span(buffer_));

    auto counters = state_->sync_get_counters();
    counters.num_pending += num_photons_;
    state_->sync_put_counters(counters);
    num_photons_ = 0;
    buffer_.clear();

    // Generate optical photons and transport to completion
    (*transport_)(*state_);
}

//---------------------------------------------------------------------------//
/*!
 * Get the accumulated action times.
 */
auto LocalOpticalGenOffload::GetActionTime() const -> MapStrDbl
{
    CELER_EXPECT(*this);
    return transport_->get_action_times(*state_->aux());
}

//---------------------------------------------------------------------------//
/*!
 * Clear local data.
 */
void LocalOpticalGenOffload::Finalize()
{
    CELER_EXPECT(*this);

    if (stream_)
    {
        // Drain and stop the consumer, then deliver any remaining hits on
        // this thread
        this->StageStreaming();
        this->WaitStreamingDrained();
        this->StopConsumer();
        this->PumpStreaming();
    }

    CELER_VALIDATE(buffer_.empty(),
                   << "offloaded photons (" << num_photons_ << " in buffer of "
                   << buffer_.size() << " distributions) were not flushed");

    auto const& accum = state_->accum();
    CELER_ASSERT(state_->aux());
    auto const& gen = generate_->counters(*state_->aux());
    CELER_LOG_LOCAL(info) << "Finalizing Celeritas after " << accum.steps
                          << " optical steps (over " << accum.step_iters
                          << " step iterations)"
                          << " from " << gen.accum.num_generated
                          << " optical photons generated from "
                          << gen.accum.buffer_size << " distributions";

    if (!gen.counters.empty())
    {
        CELER_LOG_LOCAL(warning)
            << "Not all optical photons were tracked at the end of the "
               "stepping loop: "
            << gen.counters.num_pending << " queued photons from "
            << gen.counters.buffer_size << " distributions";
    }

    // Reset all data
    *this = {};

    CELER_ENSURE(!*this);
}

//---------------------------------------------------------------------------//
// STREAMING MODE
//---------------------------------------------------------------------------//
/*!
 * Hand the buffered records to the consumer thread and return immediately.
 */
void LocalOpticalGenOffload::StageStreaming()
{
    CELER_EXPECT(*this);

    if (!stream_)
    {
        this->StartConsumer();
    }
    if (buffer_.empty())
    {
        return;
    }

    ScopedProfiling profile_this{"stage"};

    if (celeritas::device())
    {
        CELER_LOG_LOCAL(debug)
            << "Staging " << num_photons_ << " optical photons of event "
            << event_ordinal_ << " for streaming transport";
    }

    auto& sx = *stream_;
    {
        std::lock_guard<std::mutex> lock{sx.mutex};
        Streaming::Burst burst;
        burst.event = event_ordinal_ < 0 ? 0 : event_ordinal_;
        burst.photons = num_photons_;
        burst.records = std::move(buffer_);
        sx.staged.push_back(std::move(burst));
    }
    sx.cv.notify_one();

    buffer_.clear();
    num_photons_ = 0;
}

//---------------------------------------------------------------------------//
/*!
 * Deliver collected hits on the calling thread; return the drain cursor.
 */
long LocalOpticalGenOffload::PumpStreaming()
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
    }
    if (!hits.empty() && user_hit_callback_)
    {
        user_hit_callback_(make_span(hits));
    }
    return cursor;
}

//---------------------------------------------------------------------------//
/*!
 * Start the consumer thread (first stage on this stream).
 */
void LocalOpticalGenOffload::StartConsumer()
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
void LocalOpticalGenOffload::StopConsumer()
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
void LocalOpticalGenOffload::WaitStreamingDrained()
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
 * Consumer thread: a persistent transport loop fed by staged bursts.
 *
 * The loop absorbs staged bursts between step iterations, so the drain-out
 * tail of one event's photons transports the next events' instead of
 * idling. When nothing is staged, pending, or alive, the consumer publishes
 * the drain cursor and parks on the condition variable.
 */
void LocalOpticalGenOffload::ConsumerLoop()
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

    size_type const max_stall
        = transport_->params()->sim()->max_step_iters();

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
            for (auto& b : bursts)
            {
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

        if (counters.num_pending > 0 || counters.num_alive > 0)
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
            if (transport_->census_enabled()
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
                    std::lock_guard<std::mutex> lock(sx.mutex);
                    if (done > sx.drained_event)
                    {
                        sx.drained_event = done;
                    }
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
                state.accum().num_cut
                    += counters.num_active + counters.num_pending;
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
            sx.idle = true;
            ++state.accum().flushes;
            sx.cv_host.notify_all();
            if (sx.stop)
            {
                break;
            }
            sx.cv.wait(lock, [&sx] { return !sx.staged.empty() || sx.stop; });
            sx.idle = false;
        }
    }

    state.hit_sink(nullptr);
}

//---------------------------------------------------------------------------//
}  // namespace celeritas
