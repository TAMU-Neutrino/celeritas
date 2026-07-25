//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/detail/AlongStepExecutor.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "celeritas/Types.hh"
#include "celeritas/geo/CoreGeoTrackView.hh"
#include "celeritas/optical/CoreTrackView.hh"
#include "celeritas/optical/SimTrackView.hh"
#include "celeritas/optical/detail/GroupVelocityCalculator.hh"
#include "celeritas/optical/detail/OpticalKillTally.hh"
#include <cmath>
#include <cstdio>

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Complete end-of-step activity for a track.
 *
 * - Calculate the group velocity in the material
 * - Update track time based on step length and group velocity
 * - Update number of steps
 * - Update remaining MFPs to interaction
 */
struct AlongStepExecutor
{
    inline CELER_FUNCTION void operator()(CoreTrackView& track);
};

//---------------------------------------------------------------------------//
CELER_FUNCTION void AlongStepExecutor::operator()(CoreTrackView& track)
{
    auto sim = track.sim();

    CELER_ASSERT(sim.status() == TrackStatus::alive);
    CELER_ASSERT(sim.step_length() > 0);
    CELER_ASSERT(sim.post_step_action());

    // Update time
    auto group_vel = GroupVelocityCalculator{track.material_record()}(
        track.particle().energy());
    sim.add_time(sim.step_length() / group_vel);

#if !CELER_DEVICE_COMPILE
    if (track.particle().energy().value() <= 4.576e-6)
    {
        // Where visible photons spend their path. Comparing this between
        // geometry drivers says directly whether the extra bulk absorption
        // comes from longer paths, from paths in a different material, or
        // from more steps: the counts are steps per material, binned by the
        // decade of the step length.
        real_type const step = sim.step_length();
        int decade = step > 0 ? static_cast<int>(std::floor(std::log10(step)))
                              : -20;
        char buf[64];
        std::snprintf(buf,
                      sizeof(buf),
                      "pathmat mat=%u e=%d",
                      track.material_record().material_id().unchecked_get(),
                      decade < -12 ? -12 : (decade > 2 ? 2 : decade));
        celeritas::optical::detail::tally_optical_kill(
            buf, track.geometry().volume_id().unchecked_get(), false);
    }
#endif

    // Increment the step counter
    sim.increment_num_steps();

    // Kill the track if it's reached the step limit
    if (sim.num_steps() == sim.max_steps())
    {
#if !CELER_DEVICE_COMPILE
        CELER_LOG_LOCAL(debug) << "Optical track " << track.track_slot_id()
                               << " exceeded maximum step count";
#endif
        track.apply_cut();
        return;
    }

    // Update remaining MFPs to interaction
    auto phys = track.physics();
    if (sim.post_step_action() != phys.discrete_action())
    {
        // Reduce remaining mean free paths to travel. The 'discrete action'
        // case is launched separately and resets the interaction MFP itself.
        real_type mfp = phys.interaction_mfp()
                        - sim.step_length() * phys.macro_xs();
        CELER_ASSERT(mfp > 0);
        phys.interaction_mfp(mfp);
    }
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
