//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/LocalOpticalGenOffload.hh
//---------------------------------------------------------------------------//
#pragma once

#include <memory>
#include <vector>

#include "corecel/Types.hh"
#include "celeritas/Types.hh"
#include "celeritas/inp/Scoring.hh"
#include "celeritas/optical/gen/GeneratorData.hh"

#include "LocalOffloadInterface.hh"

class G4EventManager;

namespace celeritas
{
namespace detail
{
class OpticalLane;
}  // namespace detail

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
    bool Initialized() const final;

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
    // Lane-local transport state and streaming consumer
    std::shared_ptr<detail::OpticalLane> lane_;

    // Buffered distributions for offloading
    std::vector<DistributionData> buffer_;

    // Accumulated number of buffered photons
    size_type num_photons_{};

    // Number of photons to buffer before offloading
    size_type auto_flush_{};

    // Current event ID or manager for obtaining it
    UniqueEventId event_id_;
    G4EventManager* event_manager_{nullptr};

    // Set from OpticalSetupOptions::streaming at construction
    bool streaming_{false};

    // Monotonic event ordinal supplied by InitializeEvent, tagging staged
    // bursts (the caller's coordinates; under Geant4 MT each worker sees
    // an increasing subset of a global sequence)
    long event_ordinal_{-1};

    // One-shot marker for the reseed-disabled note
    bool reseed_note_logged_{false};
};

//---------------------------------------------------------------------------//
}  // namespace celeritas
