//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/surface/detail/PostBoundaryExecutor.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "corecel/math/ArrayUtils.hh"
#include "celeritas/geo/CoreGeoTrackView.hh"
#include "celeritas/optical/CoreTrackView.hh"
#include "celeritas/optical/SimTrackView.hh"
#include <cstdio>
#include "celeritas/optical/detail/OpticalKillTally.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Finalize the track's boundary crossing.
 *
 * Updates the track's state base on whether it is re-entrant in the
 * pre-volume or entrant on the post-volume. The track's surface physics
 * state will be reset.
 *
 * \note This is only called if the traversal state is "exiting", as set by
 * SurfaceInteractionApplier .
 *
 * \sa BoundaryAction
 */
struct PostBoundaryExecutor
{
    // Finalize track's boundary crossing
    inline CELER_FUNCTION void operator()(CoreTrackView&) const;
};

//---------------------------------------------------------------------------//
// INLINE DEFINITIONS
//---------------------------------------------------------------------------//
/*!
 * Finalize the track's boundary crossing.
 */
CELER_FUNCTION void PostBoundaryExecutor::operator()(CoreTrackView& track) const
{
    auto traverse = track.surface_physics().traversal();
    CELER_EXPECT(traverse.is_exiting());

#if !CELER_DEVICE_COMPILE
    if (celeritas::optical::detail::surface_trace_enabled()
        && static_cast<int>(track.track_slot_id().get())
               == celeritas::optical::detail::traced_slot().load())
    {
        char buf[160];
        std::snprintf(buf,
                      sizeof(buf),
                      "slot%d POST pos=%u in_pre=%d vol=%u",
                      static_cast<int>(track.track_slot_id().get()),
                      traverse.pos().unchecked_get(),
                      static_cast<int>(traverse.in_pre_volume()),
                      track.geometry().volume_id().unchecked_get());
        celeritas::optical::detail::trace_surface(buf);
    }
#endif

    if (traverse.in_pre_volume())
    {
        // Re-entrant into the pre-volume
        auto geo = track.geometry();
        VolumeInstanceId const before_inst = geo.volume_instance_id();
#if !CELER_DEVICE_COMPILE
        unsigned int const dbg_before = geo.volume_id().unchecked_get();
#endif
        geo.cross_boundary();

        if (before_inst && !geo.failed() && !geo.is_outside()
            && geo.volume_instance_id() == before_inst)
        {
            // The reflected photon was NOT returned to the volume it came
            // from. The navigator decides where a crossing lands by pushing
            // a fixed distance along the direction, and a reflection that
            // leaves at a shallow angle does not clear the surface in that
            // distance, so the photon stays on the wrong side -- and its
            // next step crosses the same face again, applying that surface a
            // second time. Relocate from a point displaced off the face.
            constexpr real_type clearance = 1e-7;
            Real3 const dir = geo.dir();
            Real3 pos = geo.pos();
            axpy(clearance, dir, &pos);
            geo = GeoTrackInitializer{pos, dir, {}};
        }
#if !CELER_DEVICE_COMPILE
        if (int const want = celeritas::optical::detail::traced_primary();
            want != -1 && track.sim().primary_id())
        {
            unsigned int const prim
                = track.sim().primary_id().unchecked_get();
            if (want > 0 ? prim == static_cast<unsigned int>(want)
                         : prim % static_cast<unsigned int>(-want) == 0)
            {
                // Where a reflected photon is put back. If the two drivers
                // disagree here, every later crossing is on the wrong side.
                char rbuf[160];
                std::snprintf(rbuf,
                              sizeof(rbuf),
                              "REENTER prim=%u step=%u %u -> %u "
                              "xyz=(%.6f,%.6f,%.6f)",
                              prim,
                              static_cast<unsigned int>(
                                  track.sim().num_steps()),
                              dbg_before,
                              geo.volume_id().unchecked_get(),
                              geo.pos()[0], geo.pos()[1], geo.pos()[2]);
                celeritas::optical::detail::trace_surface(rbuf);
            }
        }
#endif
        if (CELER_UNLIKELY(geo.failed()))
        {
            track.apply_errored();
            return;
        }
    }

    track.surface_physics().reset();

    if (!track.material_record().material_id())
    {
        // Kill track if it enters an invalid optical material after crossing
        // through a custom physics surface
#if !CELER_DEVICE_COMPILE
        celeritas::optical::detail::tally_optical_kill(
            "nonoptical-material",
            track.geometry().volume_id().unchecked_get(),
            track.particle().energy().value() > 4.576e-6);
#endif
        track.sim().status(TrackStatus::killed);
    }

    // Move clear of the face just crossed. When a track starts a step on a
    // boundary the navigator advances its evaluation point by a fixed push
    // and can re-detect that same face, which shows up as a spurious step
    // of push length: ~11k of them per 2 events in CCM's bulk argon, on top
    // of an otherwise correct step-length distribution. Doing this here,
    // after all surface physics has finished, is what makes it safe -- the
    // re-entrant path above still crosses from the true surface position,
    // and only a track that is leaving gets displaced. The distance is far
    // below any physically relevant length and far above the tolerance.
    if (track.sim().status() == TrackStatus::alive)
    {
        constexpr real_type clearance = 1e-7;
        auto geo = track.geometry();
        Real3 pos = geo.pos();
        axpy(clearance, geo.dir(), &pos);
        geo.move_internal(pos);
    }

    CELER_ENSURE(!track.surface_physics().is_crossing_boundary());
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
