//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/detail/PropagateExecutor.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "celeritas/Types.hh"
#include "celeritas/optical/CoreTrackView.hh"
#include "celeritas/optical/SimTrackView.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Move a track to the next interaction or geometry boundary.
 *
 * This should only apply to alive tracks.
 */
/*
 * LAUNCH BOUNDS. Without them this kernel compiles to 254 registers per thread
 * -- one short of the hardware maximum -- because VecGeom's boolean navigation
 * inlines deeply. An A30 has 65536 registers per SM, so 254 caps occupancy at
 * EIGHT warps out of a possible 64, and no launch configuration can escape it.
 * That is why raising the track-slot capacity never helped, and why deleting
 * half the solid arithmetic (the redundant daughter subtractions) changed the
 * wall clock by 0.5%: the kernel is latency-bound at an occupancy set by the
 * register file, not by the work. Capping registers to buy resident warps is
 * the trade. Swept at build time with -DCELER_OPTICAL_GEO_MIN_WARPS.
 */
#ifndef CELER_OPTICAL_GEO_BLOCK_SIZE
#    define CELER_OPTICAL_GEO_BLOCK_SIZE 256
#endif
#ifndef CELER_OPTICAL_GEO_MIN_WARPS
#    define CELER_OPTICAL_GEO_MIN_WARPS 32
#endif

struct PropagateExecutor
{
    //! Celeritas turns these into __launch_bounds__ on the generated kernel
    static constexpr int max_block_size = CELER_OPTICAL_GEO_BLOCK_SIZE;
    static constexpr int min_warps_per_eu = CELER_OPTICAL_GEO_MIN_WARPS;

    inline CELER_FUNCTION void operator()(CoreTrackView& track);
};

//---------------------------------------------------------------------------//
CELER_FUNCTION void PropagateExecutor::operator()(CoreTrackView& track)
{
    auto&& sim = track.sim();
    CELER_ASSERT(sim.status() == TrackStatus::alive);

    // Propagate up to the physics distance
    real_type step = sim.step_length();
    CELER_ASSERT(step > 0);

    auto&& geo = track.geometry();
    Propagation p = geo.find_next_step(step);
    if (p.boundary)
    {
        geo.move_to_boundary();
        sim.step_length(p.distance);
        sim.post_step_action(
            track.surface_physics().scalars().init_boundary_action);
    }
    else
    {
        CELER_ASSERT(step == p.distance);
        geo.move_internal(step);
    }
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
