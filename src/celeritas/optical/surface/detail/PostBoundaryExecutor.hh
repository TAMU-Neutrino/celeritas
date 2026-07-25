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
#include <cstdlib>
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

#if !CELER_DEVICE_COMPILE
        // Diagnostic gates: both of this file's displacements move a track
        // along its direction, which is only safe if that direction really
        // points out of the volume it is leaving. Turning each off separately
        // is how to tell which one is putting photons somewhere they should
        // not be. Host only, so CPU runs are the ones to diagnose with.
        static bool const no_reloc
            = std::getenv("CELER_NO_REENTRY_RELOC") != nullptr;
#else
        constexpr bool no_reloc = false;
#endif
#if !CELER_DEVICE_COMPILE
        if (before_inst && !geo.failed() && !geo.is_outside())
        {
            // Did the reflected photon actually get back to the volume it
            // came from? Keyed by volume so the failure can be attributed:
            // a volume whose reflections all fail is one whose photons end
            // up inside it and are absorbed there.
            bool const same = geo.volume_instance_id() == before_inst;
            bool const uv = track.particle().energy().value() > 4.576e-6;
            celeritas::optical::detail::tally_optical_kill(
                same ? "reentry-failed" : "reentry-ok", dbg_before, uv);

            // Which way the reflected photon was sent relative to the
            // surface normal. If a volume's reflections all leave along the
            // inward side, the hemisphere is being chosen against the wrong
            // orientation and the navigator cannot rescue it.
            char nb[48];
            std::snprintf(nb, sizeof(nb), "reentry-dot-%s",
                          dot_product(geo.dir(),
                                      track.surface_physics().global_normal())
                                  < 0
                              ? "neg"
                              : "pos");
            celeritas::optical::detail::tally_optical_kill(nb, dbg_before, uv);

            // Is the normal the surface physics is using even a unit vector?
            // Everything downstream -- the Fresnel angle, the reflection
            // hemisphere, and the relocation below -- assumes it is.
            real_type const nmag = norm(
                track.surface_physics().global_normal());
            celeritas::optical::detail::tally_optical_kill(
                nmag < 0.5 ? "normal-zero"
                           : (nmag < 1.5 ? "normal-unit" : "normal-big"),
                dbg_before,
                uv);
        }
#endif
        if (!no_reloc && before_inst && !geo.failed() && !geo.is_outside()
            && geo.volume_instance_id() == before_inst)
        {
            // The reflected photon was NOT returned to the volume it came
            // from. The navigator decides where a crossing lands by pushing
            // a fixed distance along the direction, and a reflection that
            // leaves at a shallow angle does not clear the surface in that
            // distance, so the photon stays on the wrong side -- and its
            // next step crosses the same face again, applying that surface a
            // second time.
            //
            // Relocate from a point displaced off the face ALONG THE NORMAL,
            // not along the direction. The clearance a displacement buys is
            // its component perpendicular to the surface, so displacing along
            // a grazing direction buys almost nothing and lands back inside
            // the same volume -- measured, that is exactly what happens for
            // the boolean-solid reflectors. The normal is the one direction
            // whose perpendicular clearance is the full step regardless of
            // incidence. Sign it to the side the photon is travelling.
            // Escalate the displacement until the volume actually changes.
            // A single fixed step cannot work for every face: how deep the
            // navigator's own push left the track depends on the incidence,
            // and a boolean solid can report a face that is not where the
            // normal says it is. The ceiling is 1e-5 cm, a hundredth of the
            // thinnest real layer in this detector (the micron TPB coatings),
            // so a displacement that helps is always physically negligible
            // and one that would not be is never taken.
            Real3 const dir = geo.dir();
            Real3 const& normal = track.surface_physics().global_normal();
            real_type const side
                = dot_product(dir, normal) < 0 ? real_type{-1} : real_type{1};
            Real3 const origin = geo.pos();
            real_type step = 1e-7;
            int attempt = 0;
            for (; attempt < 4; ++attempt, step *= 10)
            {
                Real3 pos = origin;
                axpy(side * step, normal, &pos);
                geo = GeoTrackInitializer{pos, dir, {}};
                if (geo.failed() || geo.is_outside()
                    || geo.volume_instance_id() != before_inst)
                {
                    break;
                }
            }
#if !CELER_DEVICE_COMPILE
            // How far the escalation had to go. If most crossings need the
            // largest step then the displacement is no longer negligible
            // against the micron coatings and the approach is wrong.
            char db[32];
            std::snprintf(db, sizeof(db), "reloc-attempt-%d", attempt);
            celeritas::optical::detail::tally_optical_kill(
                db, dbg_before,
                track.particle().energy().value() > 4.576e-6);
#endif
#if !CELER_DEVICE_COMPILE
            // Where the relocation actually landed, keyed by the volume the
            // photon was stuck in: "from" in the key, "to" in the volume
            // field. If it reads back the same volume the relocation is not
            // clearing the face either.
            char lb[48];
            std::snprintf(lb, sizeof(lb), "reloc-from-%u", dbg_before);
            celeritas::optical::detail::tally_optical_kill(
                lb,
                geo.is_outside() ? 0u : geo.volume_id().unchecked_get(),
                track.particle().energy().value() > 4.576e-6);
#endif
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
#if !CELER_DEVICE_COMPILE
    static bool const no_clearance
        = std::getenv("CELER_NO_CLEARANCE") != nullptr;
#else
    constexpr bool no_clearance = false;
#endif
    if (!no_clearance && track.sim().status() == TrackStatus::alive)
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
