//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalLane.hh
//! \sa OpticalLane.test.cc
//---------------------------------------------------------------------------//
#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "corecel/Types.hh"
#include "celeritas/Types.hh"
#include "celeritas/inp/Scoring.hh"
#include "celeritas/optical/gen/GeneratorData.hh"

#include "OpticalTransportLane.hh"
#include "../LocalOffloadInterface.hh"

namespace celeritas
{
namespace optical
{
class CoreStateBase;
class GeneratorAction;
class Transporter;
}  // namespace optical

namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Own the transport state and consumer thread for one optical lane.
 */
class OpticalLane final : public OpticalTransportLaneInterface
{
  public:
    //!@{
    //! \name Type aliases
    using DistributionData = optical::GeneratorDistributionData;
    using HitCallbackFunc = inp::OpticalDetector::HitCallbackFunc;
    using MapStrDbl = LocalOffloadInterface::MapStrDbl;
    //!@}

  public:
    // Construct with shared transport data and lane-local state
    OpticalLane(std::shared_ptr<optical::Transporter> transport,
                std::shared_ptr<optical::GeneratorAction const> generate,
                StreamId stream_id,
                size_type num_track_slots,
                HitCallbackFunc user_hit_callback,
                bool streaming);

    ~OpticalLane() final;

    //! Whether lane-local state has been initialized
    explicit operator bool() const { return static_cast<bool>(state_); }

    //! Access the optical transporter
    optical::Transporter& transport() const { return *transport_; }

    //! Access the optical generator
    optical::GeneratorAction const& generate() const { return *generate_; }

    //! Access lane-local transport state
    optical::CoreStateBase& state() const { return *state_; }

    // Get accumulated action times
    MapStrDbl GetActionTime() const;

    // Record the start of an event for host-side instrumentation
    void RegisterEvent(long event_ordinal);

    // Hand buffered records to the consumer thread and return immediately
    void StageStreaming(std::vector<DistributionData>& buffer,
                        size_type& num_photons,
                        long event_ordinal);

    // Deliver collected hits under the pump gate and return delivered-through
    long PumpStreaming();

    // Try to pump without blocking; return no cursor if the gate is busy
    long TryPumpStreaming();

    //! Whether the consumer thread has been started
    bool StreamingStarted() const { return static_cast<bool>(stream_); }

    // Block until everything staged has been transported to completion
    void WaitStreamingDrained();

    // Request consumer exit and join it
    void StopConsumer();

    // Log transport and streaming instrumentation at teardown
    void LogFinalization() const;

    //// SHARED SERVICE ADAPTER //

    // Transport one service burst synchronously on the owning lane thread
    OpticalTransportLaneProgress
    transport(OpticalTransportBurst const& burst) final;

    // Report an empty census after all preceding lane work has drained
    OpticalTransportLaneProgress close_event(long ordinal) final;

    // Reseed lane-local track slots before the first service event
    void reseed(long event_ordinal) final;

    // Run the continuous transport loop on the service-owned lane thread
    void run(OpticalTransportLaneControl& control) final;

    // Emit this lane's existing transport finalization line
    void finalize() final;

    // Return this lane's accumulated action times
    MapStrDbl action_time() const final { return this->GetActionTime(); }

  private:
    // Host-side streaming instrumentation (see .cc)
    struct Instrumentation;

    // Producer-consumer channel and worker thread (see .cc)
    struct Streaming;

    // Transport pending optical tracks
    std::shared_ptr<optical::Transporter> transport_;

    // Action for generating optical photons from distribution data
    std::shared_ptr<optical::GeneratorAction const> generate_;

    // Lane-local state data
    std::shared_ptr<optical::CoreStateBase> state_;

    // User hit callback, invoked from PumpStreaming on the producer thread
    HitCallbackFunc user_hit_callback_;

    // Host-side streaming instrumentation
    std::shared_ptr<Instrumentation> metrics_;

    //// EVENT-CENSUS ACCOUNTING (consumer thread only) ////

    // (event, cumulative photons staged through it), oldest first
    std::deque<std::pair<long, size_type>> staged_;
    size_type staged_photons_{0};
    // Highest event whose photons have all been generated
    long fully_generated_{-1};
    // Highest event already published as complete
    long published_event_{-1};
    // Reference ordinal for the census reduction: the oldest event not yet
    // retired, which keeps everything in flight within one ring of it
    size_type census_base_{0};

    // Drain accounting: how many staged bursts each swap of the staging
    // vector absorbed together. This -- not the producer's staging count
    // -- is what sizes a fused multi-burst append: every absorbed burst
    // is one append with its own counter round trip today.
    size_type absorb_drains_{0};
    size_type absorb_bursts_{0};
    size_type absorb_max_{0};

    // Serialize mailbox swaps and callback delivery on this lane
    std::mutex pump_mutex_;

    // Highest transport cursor whose hit callback has completed
    long delivered_through_{-1};

    // Producer-consumer state is last so its destructor joins the consumer
    // before any state referenced by that thread is destroyed
    std::shared_ptr<Streaming> stream_;

    //// STREAMING HELPERS ////

    void StartConsumer();
    void ConsumerLoop(OpticalTransportLaneControl* control = nullptr);
    long PumpStreamingImpl();
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
