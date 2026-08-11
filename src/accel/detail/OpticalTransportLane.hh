//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalTransportLane.hh
//! \sa OpticalTransportService.test.cc
//---------------------------------------------------------------------------//
#pragma once

#include <optional>
#include <vector>

#include "corecel/Types.hh"
#include "celeritas/optical/DetectorData.hh"
#include "celeritas/optical/gen/GeneratorData.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * One host-side burst submitted to an optical transport lane.
 */
struct OpticalTransportBurst
{
    long event{-1};
    size_type num_photons{0};
    size_type size_bytes{0};
    std::vector<optical::GeneratorDistributionData> records;
};

//---------------------------------------------------------------------------//
/*!
 * Hits from one event returned by an optical transport lane.
 */
struct OpticalTransportHitBatch
{
    long event{-1};
    std::vector<optical::DetectorHit> hits;
};

//---------------------------------------------------------------------------//
/*!
 * Progress returned by one host-side lane operation.
 */
struct OpticalTransportLaneProgress
{
    size_type total_generated{0};
    std::vector<OpticalTransportHitBatch> hit_batches;
    bool census_fresh{false};
    std::optional<long> min_live_ordinal;
};

//---------------------------------------------------------------------------//
/*!
 * Interface driven by one service-owned host thread.
 *
 * Implementations are single-owner: transport and close calls for one
 * instance are made from the same lane thread and never concurrently.
 * Finalize runs once after that owner thread has joined.
 */
class OpticalTransportLaneInterface
{
  public:
    virtual ~OpticalTransportLaneInterface() = default;

    // Absorb and transport one burst to the reported progress point
    virtual OpticalTransportLaneProgress
    transport(OpticalTransportBurst const& burst) = 0;

    // Finish an event after all of its bursts have been submitted
    virtual OpticalTransportLaneProgress close_event(long ordinal) = 0;

    // Emit lane-local finalization data after the owner thread has stopped
    virtual void finalize() {}
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
