//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
/*!
 * \file geocel/vg/detail/SurfNavigator.hh
 * \brief Celeritas wrapper around VecGeom's surface-model navigator.
 *
 * WHY THIS EXISTS. The solid-model navigators answer "where is the next
 * boundary" by walking a volume's solid: for CCM's PMT wavelength-shifter
 * coating that is a five-level boolean tree over nine primitives, evaluated
 * through virtual device calls, and it is 95% of GPU time. The surface model
 * answers the same question by intersecting the ray against surfaces selected
 * from a BVH, which is a different order of work.
 *
 * WHAT MAPS CLEANLY. `vgbrep::protonav::BVHSurfNavigator` operates on
 * `vecgeom::NavigationState`, the same state the solid navigators use, so the
 * scoped-temporary bridging celeritas already has for its trimmed-down state
 * (\c ScopedVgNavState) carries over unchanged.
 *
 * WHAT DOES NOT. The surface navigator splits what the solid navigator does in
 * one call: \c ComputeStepAndNextSurface reports WHICH SURFACE was hit through
 * an out parameter and leaves the state's volume path alone, and
 * \c RelocateToNextVolume needs that same surface index back to actually cross
 * it. The two calls run in separate kernels, so the index lives in the track
 * state (\c next_surf in VecgeomStateData) between them. That is the one
 * structural change this navigator forces on the rest of the geometry layer.
 *
 * The relocation is called with the position already moved onto the surface
 * and a step of zero: every use of the position inside the VecGeom crossing
 * routines is of the form `point + step * direction`, so (on-surface point, 0)
 * is identical to (query point, step) without having to carry the query point
 * across the kernel split as well.
 *
 * An earlier upstream celeritas carried this same integration (removed in
 * b3eeee4 as deprecated cleanup, original source AdePT's SurfNavigator);
 * this file follows that structure against VecGeom 2.1.1, where the
 * navigator ships in the `protonav` (prototype) namespace.
 */
//---------------------------------------------------------------------------//
#pragma once

#include <VecGeom/base/Config.h>

#ifndef VECGEOM_USE_SURF
#    error "VecGeom surface capability required to include this file"
#endif

#include <limits>
#include <VecGeom/base/Global.h>
#include <VecGeom/base/Vector3D.h>
#include <VecGeom/navigation/NavigationState.h>
#include <VecGeom/surfaces/BVHSurfNavigator.h>

#ifdef VECGEOM_ENABLE_CUDA
#    include <VecGeom/backend/cuda/Interface.h>
#endif

#include "corecel/Macros.hh"
#include "corecel/Types.hh"
#include "geocel/vg/VecgeomTypes.hh"

#include "ScopedVgNavState.hh"
#include "VgNavStateWrapper.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Navigate a VecGeom surface model, under the names the track view expects.
 */
class SurfNavigator
{
  public:
    using NavState = detail::VgNavStateWrapper;
    // NOTE the namespace: the surface navigator ships as `protonav`, a
    // PROTOTYPE, in VecGeom 2.1.1
    using Impl = vgbrep::protonav::BVHSurfNavigator<vg_real_type>;

    //-----------------------------------------------------------------------//
    /*!
     * Locate a point, filling the path.
     *
     * The surface navigator takes a placed-volume INDEX where the solid ones
     * take a pointer. The path must be cleared by the caller: the point is
     * located from `pvol_id` downward.
     */
    CELER_FUNCTION static VgPlacedVolumeInt
    LocatePointIn(VgPlacedVolumeInt pvol_id,
                  VgReal3 const& point,
                  NavState& nav,
                  bool top,
                  VgPlacedVolumeInt* exclude = nullptr)
    {
        ScopedVgNavState temp_nav{nav};
        return Impl::LocatePointIn(pvol_id, point, temp_nav, top, exclude);
    }

    //-----------------------------------------------------------------------//
    /*!
     * Find the distance to the next surface along the direction.
     *
     * \param hitsurf set to the surface that would be crossed, or left as
     *        \c vg_null_surface if none is reachable within the step limit.
     *        It must be handed back to \c RelocateToNextVolume, so the caller
     *        stores it in the track state rather than discarding it.
     *
     * The output state keeps the input state's volume path -- only the
     * boundary flag changes. Crossing happens in the relocation call.
     */
    CELER_FUNCTION static vg_real_type
    ComputeStepAndNextVolume(VgReal3 const& globalpoint,
                             VgReal3 const& globaldir,
                             vg_real_type step_limit,
                             NavState const& in_state,
                             NavState& out_state,
                             VgSurfaceInt& hitsurf)
    {
        if (step_limit <= 0)
        {
            in_state.CopyTo(&out_state);
            out_state.SetBoundaryState(false);
            return step_limit;
        }

        ScopedVgNavState temp_out_state{out_state};
        return Impl::ComputeStepAndNextSurface(globalpoint,
                                               globaldir,
                                               in_state,
                                               temp_out_state,
                                               hitsurf,
                                               step_limit);
    }

    //-----------------------------------------------------------------------//
    /*!
     * Cross the surface found by the preceding step call.
     *
     * On input \c out_state is the (pre-crossing) state written by
     * \c ComputeStepAndNextVolume; on output it is the state on the far side
     * of the surface. The crossed-surface record names the surface directly
     * -- which is what the optical boundary physics wants and what the solid
     * path reconstructs by evaluating a normal -- so its common-surface
     * index is reported through \c crossed_cs (zero when nothing crossed).
     */
    CELER_FUNCTION static void RelocateToNextVolume(VgReal3 const& globalpoint,
                                                    VgReal3 const& globaldir,
                                                    VgSurfaceInt hitsurf,
                                                    NavState& out_state,
                                                    int* crossed_cs = nullptr)
    {
        CELER_EXPECT(!out_state.IsOutside());
        CELER_EXPECT(hitsurf != vg_null_surface);
        vgbrep::CrossedSurface crossed_surf;
        ScopedVgNavState temp_out_state{out_state};
        Impl::RelocateToNextVolume(globalpoint,
                                   globaldir,
                                   vg_real_type(0),
                                   hitsurf,
                                   temp_out_state,
                                   crossed_surf);
        if (crossed_cs)
        {
            *crossed_cs = crossed_surf.hit_surf.GetCSindex();
        }
    }

    //-----------------------------------------------------------------------//
    /*!
     * Unit normal of a common surface at a point on it.
     *
     * Orientation follows the surface's own convention, which may be either
     * side of the boundary; callers reorient against the track direction,
     * exactly as with the solid path's normal.
     */
    CELER_FUNCTION static VgReal3 SurfaceNormal(int cs_index,
                                                VgReal3 const& globalpoint)
    {
        CELER_EXPECT(cs_index > 0);
        auto const& surfdata = vgbrep::SurfData<vg_real_type>::Instance();
        VgReal3 normal{0, 0, 0};
        surfdata.fCommonSurfaces[cs_index].GetNormal(
            globalpoint, normal, surfdata);
        return normal;
    }

    //-----------------------------------------------------------------------//
    //! Isotropic safety at a point inside the current volume
    CELER_FUNCTION static vg_real_type
    ComputeSafety(VgReal3 const& globalpoint,
                  NavState& state,
                  vg_real_type limit
                  = std::numeric_limits<vg_real_type>::infinity())
    {
        ScopedVgNavState temp_nav{state};
        return Impl::ComputeSafety(globalpoint, temp_nav, limit);
    }
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
