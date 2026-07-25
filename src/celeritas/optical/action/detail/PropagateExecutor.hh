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
struct PropagateExecutor
{
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

    // Most optical steps are far shorter than the distance to the nearest
    // surface: in CCM, 794k of 861k visible steps per 2 events are Mie
    // scatters inside the wavelength shifter, each ~1e-5 cm. Asking the
    // navigator to intersect the whole geometry for those is what makes
    // propagation 81% of GPU kernel time. A safety query answers "can this
    // step reach anything?" without intersecting surfaces, so when it says
    // no, the step is taken outright.
    if (!geo.is_on_boundary() && geo.find_safety(step) > step)
    {
        geo.move_internal(step);
        return;
    }

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
