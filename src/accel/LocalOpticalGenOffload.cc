//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/LocalOpticalGenOffload.cc
//---------------------------------------------------------------------------//
#include "LocalOpticalGenOffload.hh"

#include <deque>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>
#include <G4EventManager.hh>
#include <G4MTRunManager.hh>

#include "corecel/io/Logger.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/ScopedProfiling.hh"
#include "geocel/GeantUtils.hh"
#include "celeritas/optical/CoreParams.hh"
#include "celeritas/optical/CoreState.hh"
#include "celeritas/optical/Transporter.hh"
#include "celeritas/optical/Types.hh"
#include "celeritas/optical/gen/GeneratorAction.hh"

#include "SetupOptions.hh"
#include "SharedParams.hh"
#include "TimeOutput.hh"

#include "detail/OpticalLane.hh"
#include "detail/OpticalSharedQueue.hh"
#include "detail/OpticalTransportService.hh"

namespace celeritas
{
namespace
{
//---------------------------------------------------------------------------//
using OpticalService = detail::OpticalTransportService;

struct OpticalServiceRegistry
{
    std::mutex mutex;
    std::shared_ptr<OpticalService> service;
    SharedParams* owner{nullptr};
    OpticalService::Options options;
    size_type users{0};
};

//---------------------------------------------------------------------------//
OpticalServiceRegistry& optical_service_registry()
{
    static OpticalServiceRegistry result;
    return result;
}

//---------------------------------------------------------------------------//
std::shared_ptr<OpticalService> acquire_optical_service(
    SharedParams& owner,
    OpticalService::Options options,
    OpticalService::LaneFactory make_lane,
    OpticalService::HitCallback hit_callback,
    OpticalService::ActionTimeCallback action_time_callback)
{
    auto& registry = optical_service_registry();
    std::lock_guard<std::mutex> lock{registry.mutex};
    if (registry.service)
    {
        CELER_VALIDATE(registry.owner == &owner,
                       << "shared optical service belongs to a different "
                          "Celeritas problem");
        CELER_VALIDATE(
            registry.options.num_lanes == options.num_lanes
                && registry.options.unresolved_limit == options.unresolved_limit
                && registry.options.staged_bytes_limit
                       == options.staged_bytes_limit
                && registry.options.base_ordinal == options.base_ordinal
                && registry.options.log_metrics == options.log_metrics,
            << "inconsistent shared optical service options");
    }
    else
    {
        registry.service = std::make_shared<OpticalService>(
            options,
            std::move(make_lane),
            std::move(hit_callback),
            std::move(action_time_callback));
        registry.owner = &owner;
        registry.options = options;
    }
    ++registry.users;
    return registry.service;
}

//---------------------------------------------------------------------------//
void release_optical_service(std::shared_ptr<OpticalService> const& service)
{
    std::shared_ptr<OpticalService> drain;
    {
        auto& registry = optical_service_registry();
        std::lock_guard<std::mutex> lock{registry.mutex};
        CELER_ASSERT(registry.service == service);
        CELER_ASSERT(registry.users > 0);
        if (--registry.users == 0)
        {
            drain = std::move(registry.service);
            registry.owner = nullptr;
            registry.options = {};
        }
    }
    if (drain)
    {
        drain->drain_and_stop();
    }
}

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
struct LocalOpticalGenOffload::SharedProducer
{
    std::mutex mutex;
    std::shared_ptr<OpticalService> service;
    std::optional<OpticalService::ProducerToken> token;
    std::deque<long> events;
    long delivered_through{-1};
    bool barrier_cursor_pending{false};
    bool event_open{false};
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

    HitCallbackFunc user_hit_callback;
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
            user_hit_callback = options.optical->detectors.callback;
        }
    }

    // Save a pointer to the optical transporter
    auto transport = params.optical_problem_loaded().transporter;
    CELER_ASSERT(transport);

    CELER_ASSERT(transport->params());
    auto const& optical_params = *transport->params();

    // Save a pointer to the generator action
    auto generate = std::dynamic_pointer_cast<optical::GeneratorAction const>(
        params.optical_problem_loaded().generator);
    CELER_VALIDATE(generate, << "invalid optical GeneratorAction");

    // Number of optical photons to buffer before offloading
    auto const& sizes = optical_params.sizes();
    auto_flush_ = sizes.primaries;

    CELER_ASSERT(options.optical);
    auto const shared_config
        = detail::optical_shared_queue_config(options.optical->streaming);
    if (shared_config && streaming_)
    {
        CELER_VALIDATE(shared_config.lanes == sizes.streams,
                       << "shared optical lane count " << shared_config.lanes
                       << " does not match the " << sizes.streams
                       << " configured optical streams");
        CELER_VALIDATE(sizes.generators <= std::numeric_limits<size_type>::max()
                                               / sizeof(DistributionData),
                       << "optical generator byte capacity overflow");

        OpticalService::Options service_options;
        service_options.num_lanes = shared_config.lanes;
        service_options.unresolved_limit = shared_config.unresolved_limit;
        service_options.staged_bytes_limit
            = shared_config.staged_bytes_limit.value_or(
                sizes.generators * sizeof(DistributionData));
        service_options.base_ordinal = shared_config.base_ordinal;
        service_options.log_metrics = true;

        auto make_lane = [transport, generate, tracks = sizes.tracks](
                             OpticalService::LaneId lane) {
            return std::make_unique<detail::OpticalLane>(
                transport,
                generate,
                id_cast<StreamId>(*lane),
                tracks,
                HitCallbackFunc{},
                false);
        };
        OpticalService::HitCallback service_hit_callback;
        if (user_hit_callback)
        {
            service_hit_callback
                = [callback = std::move(user_hit_callback)](
                      long, std::vector<optical::DetectorHit> const& hits) {
                      if (!hits.empty())
                      {
                          callback(make_span(hits));
                      }
                  };
        }

        OpticalService::ActionTimeCallback service_action_time_callback
            = [timer = params.timer()](
                  OpticalService::LaneId lane,
                  detail::OpticalTransportLaneInterface::MapStrDbl time) {
                  timer->RecordActionTime(*lane, std::move(time));
              };

        auto service
            = acquire_optical_service(params,
                                      service_options,
                                      std::move(make_lane),
                                      std::move(service_hit_callback),
                                      std::move(service_action_time_callback));
        try
        {
            shared_state_ = std::make_shared<SharedProducer>();
            shared_state_->service = service;
            shared_state_->token.emplace(service->make_producer());
            shared_state_->delivered_through = shared_config.base_ordinal - 1;
        }
        catch (...)
        {
            auto error = std::current_exception();
            try
            {
                release_optical_service(service);
            }
            catch (...)
            {
            }
            std::rethrow_exception(error);
        }
        CELER_LOG_LOCAL(status)
            << "Enabled process-wide optical transport with "
            << shared_config.lanes << " lanes from base ordinal "
            << shared_config.base_ordinal;
        CELER_ENSURE(*this);
        return;
    }
    if (shared_config)
    {
        CELER_LOG_LOCAL(warning)
            << "Process-wide optical transport is unavailable because "
               "streaming is disabled; using the existing synchronous "
               "facade path";
    }

    // Worker-owned transport requires one optical stream per worker
    validate_geant_threading(optical_params.sizes().streams);

    auto stream_id = id_cast<StreamId>(get_geant_thread_id());
    lane_ = std::make_shared<detail::OpticalLane>(std::move(transport),
                                                  std::move(generate),
                                                  stream_id,
                                                  sizes.tracks,
                                                  std::move(user_hit_callback),
                                                  streaming_);

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
 * Whether lane-local state has been initialized.
 */
bool LocalOpticalGenOffload::Initialized() const
{
    return (lane_ && static_cast<bool>(*lane_))
           || (shared_state_ && shared_state_->service && shared_state_->token);
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

    if (this->SharedQueueEnabled())
    {
        auto& shared = *shared_state_;
        {
            std::lock_guard<std::mutex> lock{shared.mutex};
            CELER_VALIDATE(!shared.event_open,
                           << "shared optical event " << event_ordinal_
                           << " was not closed before event " << id
                           << " started");
            CELER_VALIDATE(id >= event_ordinal_,
                           << "shared optical event ordinal went backwards ("
                           << id << " after " << event_ordinal_ << ")");
        }
        shared.token->register_event(id);
        {
            std::lock_guard<std::mutex> lock{shared.mutex};
            shared.events.push_back(id);
            shared.event_open = true;
        }
        event_ordinal_ = id;
        return;
    }

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
        lane_->RegisterEvent(event_ordinal_);
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
            lane_->state().reseed(lane_->transport().params()->rng(),
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
            this->SubmitStreaming();
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

    if (this->SharedQueueEnabled())
    {
        ScopedProfiling profile_this("flush-shared-streaming");
        this->SubmitStreaming();
        this->CloseSharedEvent();
        this->PumpSharedEvents(true);
        return;
    }

    if (this->StreamingEnabled())
    {
        // Barrier with unchanged semantics: everything staged (including
        // the tail of the buffer) is transported and every hit is
        // delivered on this thread before returning
        ScopedProfiling profile_this("flush-streaming");
        this->StageStreaming();
        lane_->WaitStreamingDrained();
        lane_->PumpStreaming();
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
    lane_->generate().insert(lane_->state(), make_span(buffer_));

    auto counters = lane_->state().sync_get_counters();
    counters.num_pending += num_photons_;
    lane_->state().sync_put_counters(counters);
    num_photons_ = 0;
    buffer_.clear();

    // Generate optical photons and transport to completion
    lane_->transport()(lane_->state());
}

//---------------------------------------------------------------------------//
/*!
 * Get the accumulated action times.
 */
auto LocalOpticalGenOffload::GetActionTime() const -> MapStrDbl
{
    CELER_EXPECT(*this);
    if (this->SharedQueueEnabled())
    {
        // Lane timing is process-owned in shared mode; per-lane diagnostic
        // indexing is part of the later configuration plumbing piece.
        return {};
    }
    return lane_->GetActionTime();
}

//---------------------------------------------------------------------------//
/*!
 * Clear local data.
 */
void LocalOpticalGenOffload::Finalize()
{
    CELER_EXPECT(*this);

    if (this->SharedQueueEnabled())
    {
        std::exception_ptr failure;
        try
        {
            this->Flush();
            CELER_VALIDATE(buffer_.empty(),
                           << "offloaded photons (" << num_photons_
                           << " in buffer of " << buffer_.size()
                           << " distributions) were not flushed");
        }
        catch (...)
        {
            failure = std::current_exception();
        }

        try
        {
            this->ReleaseSharedService();
        }
        catch (...)
        {
            if (!failure)
            {
                failure = std::current_exception();
            }
        }

        *this = {};
        if (failure)
        {
            std::rethrow_exception(failure);
        }
        CELER_ENSURE(!*this);
        return;
    }

    if (lane_->StreamingStarted())
    {
        // Drain and stop the consumer, then deliver any remaining hits on
        // this thread
        this->StageStreaming();
        lane_->WaitStreamingDrained();
        lane_->StopConsumer();
        lane_->PumpStreaming();
    }

    CELER_VALIDATE(buffer_.empty(),
                   << "offloaded photons (" << num_photons_ << " in buffer of "
                   << buffer_.size() << " distributions) were not flushed");

    lane_->LogFinalization();

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
    this->SubmitStreaming();
    if (this->SharedQueueEnabled())
    {
        this->CloseSharedEvent();
    }
}

//---------------------------------------------------------------------------//
/*!
 * Try to deliver collected hits; return the delivered-through cursor.
 */
long LocalOpticalGenOffload::PumpStreaming()
{
    if (this->SharedQueueEnabled())
    {
        return this->PumpSharedEvents(false);
    }
    return lane_ ? lane_->TryPumpStreaming() : -1;
}

//---------------------------------------------------------------------------//
/*!
 * Submit buffered records without closing the current event.
 */
void LocalOpticalGenOffload::SubmitStreaming()
{
    CELER_EXPECT(*this);
    if (!this->SharedQueueEnabled())
    {
        lane_->StageStreaming(buffer_, num_photons_, event_ordinal_);
        return;
    }
    if (buffer_.empty())
    {
        return;
    }

    auto& shared = *shared_state_;
    {
        std::lock_guard<std::mutex> lock{shared.mutex};
        CELER_VALIDATE(shared.event_open,
                       << "cannot submit optical records without an open "
                          "shared event");
    }

    auto records = std::move(buffer_);
    size_type const num_photons = std::exchange(num_photons_, 0);
    buffer_.clear();
    shared.token->submit_burst(event_ordinal_, std::move(records), num_photons);
}

//---------------------------------------------------------------------------//
/*!
 * Close the current shared event against later submissions.
 */
void LocalOpticalGenOffload::CloseSharedEvent()
{
    CELER_EXPECT(this->SharedQueueEnabled());
    auto& shared = *shared_state_;
    {
        std::lock_guard<std::mutex> lock{shared.mutex};
        if (!shared.event_open)
        {
            return;
        }
    }

    shared.token->close_event(event_ordinal_);
    {
        std::lock_guard<std::mutex> lock{shared.mutex};
        shared.event_open = false;
    }
}

//---------------------------------------------------------------------------//
/*!
 * Pump owned events and return the producer-local safe cursor.
 */
long LocalOpticalGenOffload::PumpSharedEvents(bool blocking)
{
    CELER_EXPECT(this->SharedQueueEnabled());
    auto& shared = *shared_state_;

    if (!blocking)
    {
        std::lock_guard<std::mutex> lock{shared.mutex};
        if (shared.barrier_cursor_pending)
        {
            shared.barrier_cursor_pending = false;
            return shared.delivered_through;
        }
    }

    if (blocking)
    {
        std::deque<long> events;
        {
            std::lock_guard<std::mutex> lock{shared.mutex};
            events = shared.events;
        }
        for (long ordinal : events)
        {
            shared.service->wait_until_complete(ordinal);
        }
    }
    else if (shared.service->try_pump().contended)
    {
        return -1;
    }

    std::lock_guard<std::mutex> lock{shared.mutex};
    while (!shared.events.empty()
           && shared.service->is_complete(shared.events.front()))
    {
        shared.delivered_through = shared.events.front();
        shared.events.pop_front();
    }
    if (blocking)
    {
        // Flush is void, so preserve its completed cursor for the barrier's
        // following PumpStreaming call even if another producer holds the
        // service pump gate at that instant.
        shared.barrier_cursor_pending = true;
    }
    return shared.delivered_through;
}

//---------------------------------------------------------------------------//
/*!
 * Release this producer and drain the process service if it is last.
 */
void LocalOpticalGenOffload::ReleaseSharedService()
{
    CELER_EXPECT(this->SharedQueueEnabled());
    auto shared = std::move(shared_state_);
    shared->token.reset();
    release_optical_service(shared->service);
}

//---------------------------------------------------------------------------//
}  // namespace celeritas
