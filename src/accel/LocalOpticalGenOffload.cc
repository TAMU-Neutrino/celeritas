//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/LocalOpticalGenOffload.cc
//---------------------------------------------------------------------------//
#include "LocalOpticalGenOffload.hh"

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
#include "celeritas/optical/gen/GeneratorAction.hh"

#include "SetupOptions.hh"
#include "SharedParams.hh"

#include "detail/OpticalLane.hh"

namespace celeritas
{
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

    // Check the thread ID and MT model
    validate_geant_threading(optical_params.sizes().streams);

    // Save a pointer to the generator action
    auto generate = std::dynamic_pointer_cast<optical::GeneratorAction const>(
        params.optical_problem_loaded().generator);
    CELER_VALIDATE(generate, << "invalid optical GeneratorAction");

    // Number of optical photons to buffer before offloading
    auto const& sizes = optical_params.sizes();
    auto_flush_ = sizes.primaries;

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
    return lane_ && static_cast<bool>(*lane_);
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
        lane_->WaitStreamingDrained();
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
    return lane_->GetActionTime();
}

//---------------------------------------------------------------------------//
/*!
 * Clear local data.
 */
void LocalOpticalGenOffload::Finalize()
{
    CELER_EXPECT(*this);

    if (lane_->StreamingStarted())
    {
        // Drain and stop the consumer, then deliver any remaining hits on
        // this thread
        this->StageStreaming();
        lane_->WaitStreamingDrained();
        lane_->StopConsumer();
        this->PumpStreaming();
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
    lane_->StageStreaming(buffer_, num_photons_, event_ordinal_);
}

//---------------------------------------------------------------------------//
/*!
 * Deliver collected hits on the calling thread; return the drain cursor.
 */
long LocalOpticalGenOffload::PumpStreaming()
{
    return lane_ ? lane_->PumpStreaming() : -1;
}

//---------------------------------------------------------------------------//
}  // namespace celeritas
