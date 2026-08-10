//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/LocalOpticalGenOffload.hh
//---------------------------------------------------------------------------//
#pragma once

#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include "corecel/Types.hh"
#include "celeritas/Types.hh"
#include "celeritas/inp/Scoring.hh"
#include "celeritas/optical/gen/GeneratorData.hh"

#include "LocalOffloadInterface.hh"

class G4EventManager;

namespace celeritas
{
namespace optical
{
class CoreStateBase;
class GeneratorAction;
class Transporter;
}  // namespace optical

struct SetupOptions;
class SharedParams;

//---------------------------------------------------------------------------//
/*!
 * Manage offloading of optical distribution data to Celeritas.
 *
 * With \c CELER_OPTICAL_STREAMING set, the transport loop runs on a
 * persistent per-stream consumer thread instead of inside Flush: staged
 * record bursts are appended to the device queue while the loop runs, so
 * the drain-out tail of one event transports the next events' photons and
 * the device works while Geant4 tracks. Hits collected on the consumer are
 * delivered on the producer thread by PumpStreaming, and Flush becomes a
 * stage-drain-pump barrier with unchanged semantics.
 */
class LocalOpticalGenOffload final : public LocalOffloadInterface
{
  public:
    //!@{
    //! \name Type aliases
    using DistributionData = optical::GeneratorDistributionData;
    using HitCallbackFunc = inp::OpticalDetector::HitCallbackFunc;
    //!@}

  public:
    // Construct in an invalid state
    LocalOpticalGenOffload() = default;

    // Construct with shared (across threads) params
    LocalOpticalGenOffload(SetupOptions const& options, SharedParams& params);

    //!@{
    //! \name LocalOffload interface

    // Initialize with options and shared data
    void Initialize(SetupOptions const&, SharedParams&) final;

    // Set the event ID and reseed the Celeritas RNG at the start of an event
    void InitializeEvent(int) final;

    // Transport all buffered tracks to completion
    void Flush() final;

    // Clear local data and return to an invalid state
    void Finalize() final;

    // Whether the class instance is initialized
    bool Initialized() const final { return static_cast<bool>(state_); }

    // Number of buffered tracks
    size_type GetBufferSize() const final { return num_photons_; }

    // Get accumulated action times
    MapStrDbl GetActionTime() const final;
    //!@}

    // Offload optical distribution data to Celeritas
    void Push(DistributionData const&);

    //! Whether the class instance is initialized
    explicit operator bool() const { return this->Initialized(); }

    //// STREAMING MODE ////

    //! Whether streaming injection is active (see OpticalSetupOptions)
    bool StreamingEnabled() const { return streaming_; }

    // Hand the buffered records to the consumer thread, tagged with the
    // current event ordinal, and return immediately
    void StageStreaming();

    // Deliver hits collected by the consumer on the calling thread and
    // return the highest event ordinal known complete (-1 if none). The
    // ordinal counts InitializeEvent calls on this thread, zero-based; it
    // advances when the consumer fully drains its queue.
    long PumpStreaming();

  private:
    // Transport pending optical tracks
    std::shared_ptr<optical::Transporter> transport_;

    // Action for generating optical photons from distribution data
    std::shared_ptr<optical::GeneratorAction const> generate_;

    // Thread-local state data
    std::shared_ptr<optical::CoreStateBase> state_;

    // Buffered distributions for offloading
    std::vector<DistributionData> buffer_;

    // Accumulated number of buffered photons
    size_type num_photons_{};

    // Number of photons to buffer before offloading
    size_type auto_flush_{};

    // Current event ID or manager for obtaining it
    UniqueEventId event_id_;
    G4EventManager* event_manager_{nullptr};

    //// STREAMING DATA ////

    // Host-side streaming instrumentation (see .cc)
    struct Instrumentation;
    std::shared_ptr<Instrumentation> metrics_;

    // Producer-consumer channel and worker thread (see .cc)
    struct Streaming;
    std::shared_ptr<Streaming> stream_;

    // User hit callback, invoked from PumpStreaming on the producer thread
    HitCallbackFunc user_hit_callback_;

    // Set from OpticalSetupOptions::streaming at construction
    bool streaming_{false};

    // Monotonic event ordinal supplied by InitializeEvent, tagging staged
    // bursts (the caller's coordinates; under Geant4 MT each worker sees
    // an increasing subset of a global sequence)
    long event_ordinal_{-1};

    // One-shot marker for the reseed-disabled note
    bool reseed_note_logged_{false};

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

    //// STREAMING HELPERS ////

    void StartConsumer();
    void StopConsumer();
    void ConsumerLoop();
    void WaitStreamingDrained();
};

//---------------------------------------------------------------------------//
}  // namespace celeritas
