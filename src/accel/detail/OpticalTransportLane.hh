//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalTransportLane.hh
//---------------------------------------------------------------------------//
#pragma once

#include <optional>
#include <vector>

#include "corecel/Types.hh"

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
};

//---------------------------------------------------------------------------//
/*!
 * Synthetic hit payload passed through the standalone service mailbox.
 *
 * The real detector hit type will replace this test-facing aggregate when the
 * service is wired to OpticalLane in a later piece.
 */
struct OpticalTransportHit
{
    long event{-1};
    size_type num_photons{0};
};

//---------------------------------------------------------------------------//
/*!
 * Progress returned by one host-side lane operation.
 */
struct OpticalTransportLaneProgress
{
    size_type total_generated{0};
    std::vector<OpticalTransportHit> hits;
    bool census_fresh{false};
    std::optional<long> min_live_ordinal;
};

//---------------------------------------------------------------------------//
/*!
 * Interface driven by one service-owned host thread.
 *
 * Implementations are single-owner: every call for one instance is made from
 * the same lane thread and never concurrently.
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
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
