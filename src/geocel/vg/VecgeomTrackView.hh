//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file geocel/vg/VecgeomTrackView.hh
//---------------------------------------------------------------------------//
#pragma once

#include <VecGeom/base/Version.h>
// NOTE: must include Global before most other vecgeom/veccore includes
#include <VecGeom/base/Global.h>
#include <VecGeom/volumes/LogicalVolume.h>
#include <VecGeom/volumes/PlacedVolume.h>

#include "corecel/Config.hh"

#include "corecel/Macros.hh"
#include "corecel/Types.hh"
#include "corecel/cont/Span.hh"
#include "corecel/math/ArraySoftUnit.hh"
#include "corecel/math/ArrayUtils.hh"
#include "corecel/sys/ThreadId.hh"
#include "geocel/Types.hh"

#include "VecgeomData.hh"
#include "VecgeomTypes.hh"

#if CELERITAS_VECGEOM_VERSION < 0x020000
#    include "detail/BVHNavigator.hh"
#elif CELERITAS_VECGEOM_SURFACE
#    include "detail/SurfNavigator.hh"
#    if !CELER_DEVICE_COMPILE
// For CELER_DEBUG_SURF_CROSSCHECK: the solid model stays loaded alongside
// the surface model, so the same query can be put to both navigators
#        include <atomic>
#        include <cmath>
#        include <cstdio>
#        include <cstdlib>
#        include <map>
#        include <mutex>
#        include <string>

#        include "detail/SolidsNavigator.hh"
#    endif
#else
#    include "detail/SolidsNavigator.hh"
#endif

#if CELER_VGNAV == CELER_VGNAV_PATH
#    include <VecGeom/navigation/NavStatePath.h>
#else
#    include "detail/VgNavStateWrapper.hh"
#endif

#if !CELER_DEVICE_COMPILE
#    include "corecel/io/Logger.hh"
#    include "corecel/io/Repr.hh"
#    include "geocel/detail/LengthUnits.hh"
#endif

namespace celeritas
{
//---------------------------------------------------------------------------//
/*!
 * Navigate through a VecGeom geometry on a single thread.
 *
 * For a description of ordering requirements, see:
 * \sa OrangeTrackView
 *
 * \code
    VecgeomTrackView geom(vg_params_ref, vg_state_ref, trackslot_id);
   \endcode
 *
 * The "next distance" is cached as part of `find_next_step`, but it is only
 * used when the immediate next call is `move_to_boundary`.
 */
class VecgeomTrackView
{
  public:
    //!@{
    //! \name Type aliases
    using Initializer_t = GeoTrackInitializer;
    using ParamsRef = NativeCRef<VecgeomParamsData>;
    using StateRef = NativeRef<VecgeomStateData>;
#if CELERITAS_VECGEOM_VERSION < 0x020000
    using Navigator = celeritas::detail::BVHNavigator;
#elif CELERITAS_VECGEOM_SURFACE
    using Navigator = celeritas::detail::SurfNavigator;
#else
    using Navigator = celeritas::detail::SolidsNavigator;
#endif
    using ImplVolInstanceId = VgVolumeInstanceId;
    using real_type = vg_real_type;
    //!@}

  public:
    // Construct from persistent and state data
    inline CELER_FUNCTION VecgeomTrackView(
        ParamsRef const& data, StateRef const& stateview, TrackSlotId tid);

    // Initialize the state
    inline CELER_FUNCTION VecgeomTrackView&
    operator=(Initializer_t const& init);

    //// STATIC ACCESSORS ////

    //! A tiny push to make sure tracks do not get stuck at boundaries
    static CELER_CONSTEXPR_FUNCTION real_type extra_push() { return 1e-13; }

    //// ACCESSORS ////

    //!@{
    //! State accessors
    CELER_FORCEINLINE_FUNCTION Real3 const& pos() const { return pos_; }
    CELER_FORCEINLINE_FUNCTION Real3 const& dir() const { return dir_; }
    //!@}

    // Get the canonical volume ID in the current impl volume
    inline CELER_FUNCTION VolumeId volume_id() const;
    // Get the ID of the current volume instance
    inline CELER_FUNCTION VolumeInstanceId volume_instance_id() const;
    // Get the depth in the geometry hierarchy
    inline CELER_FUNCTION VolumeLevelId volume_level() const;
    // Get the volume instance ID for all levels
    inline CELER_FUNCTION void
    volume_instance_id(Span<VolumeInstanceId> levels) const;
    // Visit every volume instance in the track's path, including world
    template<class F>
    inline CELER_FUNCTION void foreach_volume_path(F&& visit) const;

    // Get the current volume's ID
    inline CELER_FUNCTION ImplVolumeId impl_volume_id() const;
    // The current surface ID
    inline CELER_FUNCTION ImplSurfaceId impl_surface_id() const;
    // After 'find_next_step', the next straight-line surface
    inline CELER_FUNCTION ImplSurfaceId next_impl_surface_id() const;

    // Whether the track is outside the valid geometry region
    CELER_FORCEINLINE_FUNCTION bool is_outside() const;
    // Whether the track is exactly on a surface
    CELER_FORCEINLINE_FUNCTION bool is_on_boundary() const;
    //! Whether the last operation resulted in an error
    CELER_FORCEINLINE_FUNCTION bool failed() const { return failed_; }
    // Get the normal vector of the current surface
    inline CELER_FUNCTION Real3 normal() const;

    //// OPERATIONS ////

    // Find the distance to the next boundary, up to and including a step
    inline CELER_FUNCTION Propagation find_next_step(real_type max_step);

    // Find the safety at the current position (infinite max)
    inline CELER_FUNCTION real_type find_safety();

    // Find the safety at the current position up to a maximum step distance
    inline CELER_FUNCTION real_type find_safety(real_type max_step);

    // Move to the boundary in preparation for crossing it
    inline CELER_FUNCTION void move_to_boundary();

    // Move within the volume
    inline CELER_FUNCTION void move_internal(real_type step);

    // Move within the volume to a specific point
    inline CELER_FUNCTION void move_internal(Real3 const& pos);

    // Cross from one side of the current surface to the other
    inline CELER_FUNCTION void cross_boundary();

    // Change direction
    inline CELER_FUNCTION void set_dir(Real3 const& newdir);

#if !CELER_DEVICE_COMPILE
    //! Debug: current position in the frame of the deepest volume instance
    Real3 debug_local_pos() const
    {
        vecgeom::Transformation3D trans;
        vgstate_.TopMatrix(trans);
        auto local = trans.Transform(to_vgvector(pos_));
        return {local[0], local[1], local[2]};
    }

    //! Debug: current direction in the frame of the deepest volume instance
    Real3 debug_local_dir() const
    {
        vecgeom::Transformation3D trans;
        vgstate_.TopMatrix(trans);
        auto local = trans.TransformDirection(to_vgvector(dir_));
        return {local[0], local[1], local[2]};
    }
#endif

  private:
    //// TYPES ////

    using VgLogVol = VgLogicalVolume<MemSpace::native>;
    using VgPlacedVol = VgPlacedVolume<MemSpace::native>;

#if CELER_VGNAV == CELER_VGNAV_PATH
    using NavStateWrapper = vecgeom::NavStatePath&;
#else
    using NavStateWrapper = detail::VgNavStateWrapper;
#endif

    //// DATA ////

    //! Shared/persistent geometry data
    ParamsRef const& params_;
    StateRef const& state_;
    TrackSlotId tid_;

    //!@{
    //! Referenced thread-local data
    NavStateWrapper vgstate_;
    NavStateWrapper vgnext_;
    Real3& pos_;
    Real3& dir_;
    //! Centre and radius of the cached "no boundary within" sphere
    Real3& safety_pos_;
    ::celeritas::real_type& safety_radius_;
    int& safety_credit_;
    //! Surface hit by find_next_step, consumed by cross_boundary
    VgSurfaceInt* next_surf_{nullptr};
    //! Navigation state saved before the last crossing, for reflections
    VgNavStateImpl* pre_cross_state_{nullptr};

    //!@}

    // Temporary data
    real_type next_step_{0};
    bool failed_{false};
    //! Normal of the surface crossed by this track view's last crossing
    Real3 normal_{0, 0, 0};

    //// HELPER FUNCTIONS ////

    // Calculate the outward normal of a state's volume at the current point
    inline CELER_FUNCTION bool
    calc_normal(NavStateWrapper const& state, Real3* normal) const;

    // Estimate that normal from the solid's signed distance field
    inline CELER_FUNCTION bool
    calc_normal_gradient(NavStateWrapper const& state, Real3* normal) const;

    // Whether any next distance-to-boundary has been found
    inline CELER_FUNCTION bool has_next_step() const;

    // Whether the next distance-to-boundary is to a surface
    inline CELER_FUNCTION bool is_next_boundary() const;

    // Get a reference to the current volume instance
    inline CELER_FUNCTION VgPlacedVol const& physical_volume() const;

    // Get a reference to the current volume
    inline CELER_FUNCTION VgLogVol const& logical_volume() const;
};

//---------------------------------------------------------------------------//
// INLINE DEFINITIONS
//---------------------------------------------------------------------------//
/*!
 * Construct from persistent and state data.
 */
CELER_FUNCTION
VecgeomTrackView::VecgeomTrackView(
    ParamsRef const& params, StateRef const& states, TrackSlotId tid)
    : params_(params)
    , state_(states)
    , tid_(tid)
#if CELER_VGNAV == CELER_VGNAV_PATH
    // Nav path holds direct references to state with unused "last state"
    , vgstate_{states.state[tid]}
    , vgnext_{states.next_state[tid]}
#else
    , vgstate_{states.state[tid], states.boundary[tid]}
    , vgnext_{states.next_state[tid], states.next_boundary[tid]}
#endif
    , pos_(states.pos[tid])
    , dir_(states.dir[tid])
    , safety_pos_(states.safety_pos[tid])
    , safety_radius_(states.safety_radius[tid])
    , safety_credit_(states.safety_credit[tid])
{
    if constexpr (CELERITAS_VECGEOM_SURFACE)
    {
        next_surf_ = &states.next_surf[tid];
        pre_cross_state_ = &states.pre_cross_state[tid];
    }
}

//---------------------------------------------------------------------------//
/*!
 * Construct the state.
 *
 * If a valid parent ID is provided, the state is constructed from a direction
 * and a copy of the parent state.  This is a faster method of creating
 * secondaries from a parent that has just been absorbed, or when filling in an
 * empty track from a parent that is still alive.
 *
 * Otherwise, the state is initialized from a starting location and direction,
 * which is expensive.
 */
CELER_FUNCTION VecgeomTrackView&
VecgeomTrackView::operator=(Initializer_t const& init)
{
    CELER_EXPECT(is_soft_unit_vector(init.dir));
    failed_ = false;

    // Initialize direction
    dir_ = init.dir;

    if constexpr (CELERITAS_VECGEOM_SURFACE)
    {
        // A fresh track has no pending surface crossing
        *next_surf_ = vg_null_surface;
    }

    if (init.parent)
    {
        // Copy the navigation state and position from the parent state
        if (tid_ != init.parent)
        {
            VecgeomTrackView other(params_, state_, init.parent);
            other.vgstate_.CopyTo(&vgstate_);
            pos_ = other.pos_;
        }
        // Set up the next state and initialize the direction
        vgnext_ = vgstate_;

        CELER_ENSURE(this->pos() == init.pos);
        CELER_ENSURE(!this->has_next_step());
        return *this;
    }

    // Initialize the state from a position
    pos_ = init.pos;

    // Set up current state and locate daughter volume
    vgstate_.Clear();
#if CELERITAS_VECGEOM_SURFACE
    // The surface navigator takes a placed-volume index, not a pointer
    VgPlacedVolumeInt world = vecgeom::NavigationState::WorldId();
#else
    auto const* world = params_.scalars.world<MemSpace::native>();
#endif
    // LocatePointIn sets `vgstate_`
    constexpr bool contains_point = true;
    Navigator::LocatePointIn(
        world, to_vgvector(pos_), vgstate_, contains_point);

    if (CELER_UNLIKELY(vgstate_.IsOutside()))
    {
#if !CELER_DEVICE_COMPILE
        auto msg = CELER_LOG_LOCAL(error);
        msg << "Failed to initialize geometry state at " << repr(pos_) << ' '
            << lengthunits::native_label;
#endif
        failed_ = true;
    }

    return *this;
}

//---------------------------------------------------------------------------//
/*!
 * Get the volume ID in the current cell.
 */
CELER_FORCEINLINE_FUNCTION VolumeId VecgeomTrackView::volume_id() const
{
    CELER_EXPECT(!this->is_outside());
    CELER_EXPECT(!params_.volumes.empty());

    return params_.volumes[this->impl_volume_id()];
}

//---------------------------------------------------------------------------//
/*!
 * Get the physical volume ID in the current cell.
 *
 * If built with Geant4, this is the canonical volume instance ID. If built
 * with VGDML, this is an "implementation" instance ID.
 */
CELER_FUNCTION VolumeInstanceId VecgeomTrackView::volume_instance_id() const
{
    CELER_EXPECT(!this->is_outside());
    auto ipv_id = id_cast<ImplVolInstanceId>(this->physical_volume().id());
    return params_.volume_instances[ipv_id];
}

//---------------------------------------------------------------------------//
/*!
 * Get the depth in the geometry hierarchy.
 */
CELER_FUNCTION VolumeLevelId VecgeomTrackView::volume_level() const
{
    CELER_EXPECT(!this->is_outside());
    auto result = id_cast<VolumeLevelId>(vgstate_.GetLevel());
    CELER_ENSURE(result < params_.scalars.num_volume_levels);
    return result;
}

//---------------------------------------------------------------------------//
/*!
 * Get the volume instance ID at each volume level.
 */
CELER_FUNCTION void
VecgeomTrackView::volume_instance_id(Span<VolumeInstanceId> levels) const
{
    this->foreach_volume_path(
        [levels](VolumeLevelId lev, VolumeInstanceId vol_inst) {
            CELER_EXPECT(lev < levels.size());
            CELER_EXPECT(vol_inst);
            levels[*lev] = vol_inst;
        });
}

//---------------------------------------------------------------------------//
/*!
 * Apply the function with the volume instance ID and level.
 *
 * This can be used to construct a unique volume instance ID or fill a vector
 * with volume levels. It is performed in global-to-local order.
 */
template<class F>
CELER_FUNCTION void VecgeomTrackView::foreach_volume_path(F&& visit) const
{
    for (auto lev : range(this->volume_level() + 1))
    {
        VgPlacedVol const* pv = vgstate_.At(*lev);
        CELER_ASSERT(pv);
        auto ipv_id = id_cast<ImplVolInstanceId>(pv->id());
        visit(lev, params_.volume_instances[ipv_id]);
    }
}

//---------------------------------------------------------------------------//
/*!
 * Get the volume ID in the current cell.
 */
CELER_FORCEINLINE_FUNCTION ImplVolumeId VecgeomTrackView::impl_volume_id() const
{
    CELER_EXPECT(!this->is_outside());
    return id_cast<ImplVolumeId>(this->logical_volume().id());
}

//---------------------------------------------------------------------------//
/*!
 * The current surface frame ID.
 */
CELER_FUNCTION ImplSurfaceId VecgeomTrackView::impl_surface_id() const
{
    return {};
}

//---------------------------------------------------------------------------//
/*!
 * After 'find_next_step', the next straight-line surface.
 */
CELER_FUNCTION ImplSurfaceId VecgeomTrackView::next_impl_surface_id() const
{
    return {};
}

//---------------------------------------------------------------------------//
/*!
 * Whether the track is outside the valid geometry region.
 */
CELER_FUNCTION bool VecgeomTrackView::is_outside() const
{
    return vgstate_.IsOutside();
}

//---------------------------------------------------------------------------//
/*!
 * Whether the track is on the boundary of a volume.
 */
CELER_FUNCTION bool VecgeomTrackView::is_on_boundary() const
{
    return vgstate_.IsOnBoundary();
}

//---------------------------------------------------------------------------//
/*!
 * Get the surface normal of the boundary the track is currently on.
 *
 * The normal is evaluated during \c cross_boundary , while both the
 * pre-crossing and post-crossing states are available: after the crossing the
 * volume being exited is no longer reachable. If no crossing has been
 * performed by this track view (e.g. the track was moved to a boundary but not
 * yet crossed) the normal is evaluated from the current volume.
 *
 * The result points out of whichever volume owns the surface, which may be
 * either side of the boundary; callers that need a specific orientation (such
 * as optical surface physics) reorient it against the track direction.
 */
CELER_FUNCTION Real3 VecgeomTrackView::normal() const
{
    CELER_EXPECT(this->is_on_boundary());

    if (normal_ != Real3{0, 0, 0})
    {
        return normal_;
    }

    // Prefer a normal the solid VOUCHES FOR. Both calls write a unit vector
    // whether or not the point is on that solid's surface, and a normal the
    // solid disclaims is not merely imprecise: sampled against Geant4 on the
    // intersection solids in this geometry, of 1406 disclaimed normals only
    // 8 were correct, 269 were inverted and 1129 were more than 60 degrees
    // off. The old code took the second answer unconditionally, so a
    // disclaimed-but-correct normal from the current volume was routinely
    // replaced by a disclaimed-and-wrong one from the other.
    Real3 from_state{0, 0, 0};
    if (this->calc_normal(vgstate_, &from_state))
    {
        return from_state;
    }
    Real3 from_next{0, 0, 0};
    if (this->calc_normal(vgnext_, &from_next))
    {
        return from_next;
    }
    // Neither solid claims the point, so neither analytic answer is worth
    // anything. Estimate it from the signed-distance gradient instead. That
    // estimate reports failure for a state that does not own the surface, so
    // asking in the same order picks the owner without a separate test.
    Real3 from_grad{0, 0, 0};
    if (this->calc_normal_gradient(vgstate_, &from_grad)
        || this->calc_normal_gradient(vgnext_, &from_grad))
    {
        return from_grad;
    }
    // Even the gradient is flat. Keep the current volume's answer if it
    // produced one: it is at least the surface the track is leaving.
    return from_state != Real3{0, 0, 0} ? from_state : from_next;
}

//---------------------------------------------------------------------------//
/*!
 * Calculate the outward normal of a state's volume at the current position.
 *
 * The point is transformed into the frame of the state's deepest volume, the
 * solid is asked for its normal there, and the result is rotated back into the
 * global frame. Returns whether the position is actually on that solid's
 * surface: it is not, for instance, when the track is entering a daughter,
 * where the position lies strictly inside the mother.
 */
CELER_FUNCTION bool VecgeomTrackView::calc_normal(NavStateWrapper const& state,
                                                  Real3* normal) const
{
    CELER_EXPECT(normal);
    auto const* pv = state.Top();
    if (!pv)
    {
        return false;
    }

    // Transform the global point into the volume's own frame
    vecgeom::Transformation3D trans;
    state.TopMatrix(trans);
    auto local_pos = trans.Transform(to_vgvector(pos_));

    // The unplaced volume works in that same frame; VPlacedVolume::Normal
    // would apply the placement transform a second time
    VgReal3 local_normal{0, 0, 0};
    bool on_surface
        = pv->GetUnplacedVolume()->Normal(local_pos, local_normal);
    if (local_normal.Mag2() == 0)
    {
        // Solid could not supply a normal (e.g. an interior point)
        return false;
    }

    auto global_normal = trans.InverseTransformDirection(local_normal);
    (*normal)[0] = global_normal[0];
    (*normal)[1] = global_normal[1];
    (*normal)[2] = global_normal[2];
    *normal = make_unit_vector(*normal);
    return on_surface;
}

//---------------------------------------------------------------------------//
/*!
 * Estimate a state's surface normal from its solid's signed distance field.
 *
 * This is for the case \c calc_normal cannot serve: VecGeom disclaims its
 * analytic normal at most surface points of a converted boolean solid, and a
 * disclaimed normal is not merely imprecise but essentially random. The signed
 * distance is defined everywhere -- negative inside, positive outside -- and
 * its gradient is the outward normal, so probe it on the four corners of a
 * tetrahedron about the point. That is four evaluations where a central
 * difference would take six.
 *
 * The magnitude is capped at the probe radius, which does two things.
 * VecGeom's boolean safeties are not always a distance -- the union kernel
 * hands back -kTolerance as an "invalid side" marker -- and the cap stops such
 * a value from inverting the result, which measured is what it otherwise does
 * on nine surface points in ten. It also makes the estimate self-selecting:
 * for a point farther than the probe radius from THIS solid's surface every
 * corner saturates to the same value and the gradient vanishes, so a state
 * that does not own the surface reports failure rather than the direction to
 * its own nearest wall.
 *
 * Scored against Geant4 at conversion time over 408,000 points that VecGeom
 * disclaims: 94% within 8 degrees, none inverted, none degenerate. On the
 * intersection solids where CCM's residual light deficit lives it is exact.
 */
CELER_FUNCTION bool
VecgeomTrackView::calc_normal_gradient(NavStateWrapper const& state,
                                       Real3* normal) const
{
    CELER_EXPECT(normal);
    auto const* pv = state.Top();
    if (!pv)
    {
        return false;
    }

    // Transform the global point into the volume's own frame, as calc_normal
    // does: the unplaced volume works in that frame
    vecgeom::Transformation3D trans;
    state.TopMatrix(trans);
    auto local_pos = trans.Transform(to_vgvector(pos_));
    auto const* solid = pv->GetUnplacedVolume();

    // The four alternating vertices of a cube, so the offsets span all three
    // axes in four evaluations rather than six
    real_type const corner[4][3]
        = {{1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {1, 1, 1}};
    // A nanometre: three orders of magnitude above VecGeom's surface
    // tolerance and three below the thinnest real feature in this detector
    real_type const probe_step = 1e-7;
    real_type const max_dist = probe_step * real_type{1.732050807568877};

    real_type grad[3] = {0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
        auto probe = local_pos;
        for (int ax = 0; ax < 3; ++ax)
        {
            probe[ax] = local_pos[ax] + probe_step * corner[i][ax];
        }

        // Only one of the two safeties is defined at a given point
        bool inside = solid->Contains(probe);
        real_type dist
            = inside ? solid->SafetyToOut(probe) : solid->SafetyToIn(probe);
        dist = min(dist < 0 ? -dist : dist, max_dist);

        for (int ax = 0; ax < 3; ++ax)
        {
            grad[ax] += (inside ? -dist : dist) * corner[i][ax];
        }
    }

    VgReal3 local_normal{grad[0], grad[1], grad[2]};
    if (local_normal.Mag2() == 0)
    {
        // Flat across the probe: this solid does not own the surface
        return false;
    }

    auto global_normal = trans.InverseTransformDirection(local_normal);
    (*normal)[0] = global_normal[0];
    (*normal)[1] = global_normal[1];
    (*normal)[2] = global_normal[2];
    *normal = make_unit_vector(*normal);
    return true;
}

//---------------------------------------------------------------------------//
/*!
 * Find the distance to the next geometric boundary.
 */
CELER_FUNCTION Propagation VecgeomTrackView::find_next_step(real_type max_step)
{
    CELER_EXPECT(!this->is_outside());
    CELER_EXPECT(max_step > 0);

#if CELERITAS_VECGEOM_VERSION < 0x020000
    bool const use_cache = params_.scalars.use_safety_cache;
#else
    // The surface-model navigator does not carry the fused safety
    constexpr bool use_cache = false;
#endif

    if (use_cache && safety_radius_ > 0 && !vgstate_.IsOnBoundary())
    {
        // No boundary lies within safety_radius_ of safety_pos_, so a step
        // that stays inside that sphere cannot reach one. Unlike the cached
        // next_step_, which set_dir throws away, the sphere is isotropic and
        // survives every scattering event -- which is what makes it worth
        // anything in a wavelength shifter, where a photon changes direction
        // on every step and takes tens of them to cross a two-micron coating.
        // |pos - centre| + max_step < radius, without the square root: this
        // runs on every step, so the comparison is squared instead
        ::celeritas::real_type const slack = safety_radius_ - max_step;
        ::celeritas::real_type dist_sq{0};
        for (int i = 0; i < 3; ++i)
        {
            ::celeritas::real_type const d = pos_[i] - safety_pos_[i];
            dist_sq += d * d;
        }
        if (slack > 0 && dist_sq < slack * slack)
        {
            next_step_ = max_step;
            // The sphere paid for itself here, so keep buying them
            safety_credit_ = 4;
            // Only is_next_boundary() reads vgnext_ on this path, and the
            // caller will move internally rather than cross
            vgnext_.SetBoundaryState(false);

            Propagation result;
            result.distance = max_step;
            result.boundary = false;
            return result;
        }
    }

    // A safety costs about as much as the step query it rides along with, so
    // it must not be bought where it cannot be used. A track crossing bulk
    // argon reaches a boundary on nearly every step and can never take the
    // fast path; a track diffusing inside a two-micron coating takes it
    // repeatedly. The credit tells them apart at runtime: it is reset on every
    // crossing so each new volume is probed once, set high whenever a sphere
    // actually gets used, and decays otherwise until this track stops paying.
    // This changes only WHEN a safety is computed, never what the query
    // returns, so the output is unaffected.
    bool const want_safety = use_cache && safety_credit_ >= 0
                             && !vgstate_.IsOnBoundary();

    // TODO: vgnext is simply copied and the boundary flag optionally set
    vg_real_type safety{0};
#if CELERITAS_VECGEOM_VERSION < 0x020000
    next_step_ = Navigator::ComputeStepAndNextVolume(to_vgvector(pos_),
                                                     to_vgvector(dir_),
                                                     max_step,
                                                     vgstate_,
                                                     vgnext_,
                                                     want_safety ? &safety
                                                                 : nullptr);
#elif CELERITAS_VECGEOM_SURFACE
    *next_surf_ = vg_null_surface;
    next_step_ = Navigator::ComputeStepAndNextVolume(to_vgvector(pos_),
                                                     to_vgvector(dir_),
                                                     max_step,
                                                     vgstate_,
                                                     vgnext_,
                                                     *next_surf_);
    // The index and the flag must agree: is_next_boundary() reads the flag,
    // and cross_boundary consumes the index
    CELER_ASSERT((*next_surf_ != vg_null_surface) == vgnext_.IsOnBoundary());
    if (vgstate_.IsOnBoundary() && *next_surf_ != vg_null_surface
        && next_step_ < real_type(1e-7))
    {
        // A surface reported within 1e-7 cm of a just-crossed boundary is
        // the crossed face re-reported from its far side: the thinnest real
        // feature in this geometry sits 2e-4 cm away, three orders above
        // this floor. Honoring the phantom hit ping-pongs the photon across
        // the same face until the iteration cap, with the device idling at
        // 0.02% occupancy for tens of thousands of iterations. Report no
        // boundary within the step instead; the true boundary is found by
        // the next query from inside the volume.
        *next_surf_ = vg_null_surface;
        vgnext_.SetBoundaryState(false);
        next_step_ = max_step;
    }
#    if !CELER_DEVICE_COMPILE
    {
        // CELER_DEBUG_SURF_CROSSCHECK: put the same query to the solid
        // navigator and tally, by volume, whether the two agree on the
        // distance to the next boundary. Diagnostic for the coating-dwell
        // anomaly; costs a second navigation per step when enabled.
        static bool const crosscheck
            = std::getenv("CELER_DEBUG_SURF_CROSSCHECK") != nullptr;
        if (crosscheck && !vgstate_.IsOnBoundary())
        {
            VgNavStateImpl tmp_impl{};
            VgBoundary tmp_b{};
            detail::VgNavStateWrapper tmp_next{tmp_impl, tmp_b};
            vg_real_type solid_step
                = detail::SolidsNavigator::ComputeStepAndNextVolume(
                    to_vgvector(pos_),
                    to_vgvector(dir_),
                    max_step,
                    vgstate_,
                    tmp_next);
            bool surf_hit = (*next_surf_ != vg_null_surface);
            bool solid_hit = tmp_next.IsOnBoundary();
            char const* rel = nullptr;
            if (surf_hit == solid_hit)
            {
                vg_real_type diff = std::fabs(next_step_ - solid_step);
                rel = (!surf_hit || diff < 1e-7) ? "agree"
                      : next_step_ > solid_step  ? "surf-longer"
                                                 : "surf-shorter";
            }
            else
            {
                rel = surf_hit ? "solid-miss" : "surf-miss";
                if (!surf_hit)
                {
                    // Sample where the surface model misses a boundary the
                    // solid model sees: LOCAL position and direction plus
                    // the solid model's distance identify the missed face
                    static std::atomic<int> miss_budget{80};
                    if (miss_budget.fetch_sub(1) > 0)
                    {
                        auto lp = this->debug_local_pos();
                        auto ld = this->debug_local_dir();
                        std::fprintf(
                            stderr,
                            "[NAVMISS] vol=%u lp=(%.6f,%.6f,%.6f) lr=%.5f "
                            "ld=(%.4f,%.4f,%.4f) solid=%.3e max=%.3e\n",
                            this->volume_id().unchecked_get(),
                            lp[0],
                            lp[1],
                            lp[2],
                            std::sqrt(lp[0] * lp[0] + lp[1] * lp[1]
                                      + lp[2] * lp[2]),
                            ld[0],
                            ld[1],
                            ld[2],
                            static_cast<double>(solid_step),
                            static_cast<double>(max_step));
                    }
                }
            }
            struct NavDiffTally
            {
                std::mutex mutex;
                std::map<std::string, long> counts;
                ~NavDiffTally()
                {
                    for (auto const& kv : counts)
                    {
                        std::fprintf(stderr,
                                     "[NAVDIFF] %s %ld\n",
                                     kv.first.c_str(),
                                     kv.second);
                    }
                }
            };
            static NavDiffTally tally;
            char buf[64];
            std::snprintf(buf,
                          sizeof(buf),
                          "%s vol=%u",
                          rel,
                          this->volume_id().unchecked_get());
            std::lock_guard<std::mutex> lock(tally.mutex);
            ++tally.counts[buf];
        }
    }
#    endif
#else
    next_step_ = Navigator::ComputeStepAndNextVolume(
        to_vgvector(pos_), to_vgvector(dir_), max_step, vgstate_, vgnext_);
#endif
    if (want_safety)
    {
        // Recentre the sphere here whether or not a safety came back: a zero
        // radius disables the fast path, which is the right answer on a
        // boundary and for the solids whose safety is only a marker
        safety_pos_ = pos_;
        safety_radius_ = safety;
    }
    if (use_cache)
    {
        safety_credit_ = max(safety_credit_ - 1, -4);
    }

    next_step_ = max(next_step_, this->extra_push());

    if (!this->is_next_boundary())
    {
        // Soft equivalence between distance and max step is because the
        // BVH navigator subtracts and then re-adds a bump distance to the
        // step
        CELER_ASSERT(soft_equal(next_step_, max(max_step, this->extra_push())));
        next_step_ = max_step;
    }

    Propagation result;
    result.distance = next_step_;
    result.boundary = this->is_next_boundary();

    CELER_ENSURE(this->has_next_step());
    CELER_ENSURE(result.distance > 0);
    CELER_ENSURE(result.distance <= max(max_step, this->extra_push()));
    CELER_ENSURE(result.boundary || result.distance == max_step
                 || max_step < this->extra_push());
    return result;
}

//---------------------------------------------------------------------------//
/*!
 * Find the safety at the current position.
 */
CELER_FUNCTION real_type VecgeomTrackView::find_safety()
{
    return this->find_safety(vecgeom::kInfLength);
}

//---------------------------------------------------------------------------//
/*!
 * Find the safety at the current position up to a maximum distance.
 *
 * The safety within a step is only needed up to the end of the physics step
 * length.
 */
CELER_FUNCTION real_type VecgeomTrackView::find_safety(real_type max_radius)
{
    CELER_EXPECT(!this->is_outside());
    CELER_EXPECT(!this->is_on_boundary());
    CELER_EXPECT(max_radius > 0);

    real_type safety = Navigator::ComputeSafety(
        to_vgvector(this->pos()), vgstate_, max_radius);
    safety = min<real_type>(safety, max_radius);

    // Since the reported "safety" is negative if we've moved slightly beyond
    // the boundary of a solid without crossing it, we must clamp to zero.
    return max<real_type>(safety, 0);
}

//---------------------------------------------------------------------------//
/*!
 * Move to the next boundary but don't cross yet.
 */
CELER_FUNCTION void VecgeomTrackView::move_to_boundary()
{
    CELER_EXPECT(this->has_next_step());
    CELER_EXPECT(this->is_next_boundary());

    // Move next step
    axpy(next_step_, dir_, &pos_);
    next_step_ = 0;
    vgstate_.SetBoundaryState(true);

    CELER_ENSURE(this->is_on_boundary());
}

//---------------------------------------------------------------------------//
/*!
 * Cross from one side of the current surface to the other.
 *
 * The position *must* be on the boundary following a move-to-boundary.
 */
CELER_FUNCTION void VecgeomTrackView::cross_boundary()
{
    CELER_EXPECT(!this->is_outside());
    CELER_EXPECT(this->is_on_boundary());
    CELER_EXPECT(this->is_next_boundary());

#if CELERITAS_VECGEOM_SURFACE
    Real3 surf_normal{0, 0, 0};
    if (*next_surf_ != vg_null_surface && vgnext_.Top() != nullptr)
    {
        // Cross the surface found by find_next_step: this is where the
        // volume path actually changes on the surface model. The index
        // names a surface of the PRE-crossing volume, so it is consumed
        // here -- and the state being left is saved first, because if the
        // boundary physics reflects the photon, the crossing must be
        // undone.
        *pre_cross_state_ = vgstate_.GetState();
        int crossed_cs{0};
        Navigator::RelocateToNextVolume(to_vgvector(this->pos_),
                                        to_vgvector(this->dir_),
                                        *next_surf_,
                                        vgnext_,
                                        &crossed_cs);
        *next_surf_ = vg_null_surface;
        if (crossed_cs > 0)
        {
            // The crossed surface names its own normal: exact, no solid
            // interrogation, no gradient estimate
            auto n = Navigator::SurfaceNormal(crossed_cs,
                                              to_vgvector(this->pos_));
            surf_normal = {n[0], n[1], n[2]};
        }
    }
    else
    {
        // Second crossing without an intervening find_next_step: the
        // optical reflected re-entry. The photon returns to the volume it
        // came from, and the saved pre-crossing state IS that volume --
        // restoring it is exact where the solid path's displaced
        // relocation is approximate, and costs no navigation call.
        state_.state[tid_] = *pre_cross_state_;
        state_.next_state[tid_] = *pre_cross_state_;
        vgstate_.SetBoundaryState(true);
        vgnext_.SetBoundaryState(true);
    }
#else
    // Relocate to next tracking volume (maybe across multiple boundaries)
    if (vgnext_.Top() != nullptr)
    {
        Navigator::RelocateToNextVolume(
            to_vgvector(this->pos_), to_vgvector(this->dir_), vgnext_);
    }
#endif

    // Evaluate the crossed surface's normal while the volume being exited is
    // still reachable: the position is on its surface unless the track is
    // entering one of its daughters, in which case the entered volume owns
    // the surface
    // Prefer a normal the solid VOUCHES FOR: calc_normal writes a unit
    // vector whether or not the point is on that solid's surface, and taking
    // the second answer unconditionally replaces a disclaimed-but-correct
    // normal with a disclaimed-and-wrong one. Sampled against Geant4 on this
    // geometry's boolean solids, of 1406 disclaimed normals only 8 were
    // correct: 269 were inverted and 1129 were more than 60 degrees off.
    // This is the value everything downstream steers by.
    normal_ = Real3{0, 0, 0};
#if CELERITAS_VECGEOM_SURFACE
    if (surf_normal != Real3{0, 0, 0})
    {
        // The surface model already identified the crossed surface; its
        // analytic normal supersedes the solid-interrogation chain below
        normal_ = make_unit_vector(surf_normal);
    }
    else
#endif
    if (!this->calc_normal(vgstate_, &normal_))
    {
        Real3 from_next{0, 0, 0};
        if (this->calc_normal(vgnext_, &from_next))
        {
            normal_ = from_next;
        }
        else
        {
            // Neither solid claims the point -- for the boolean solids in
            // this geometry that is the majority of crossings -- so estimate
            // the normal from the signed-distance gradient. That estimate
            // reports failure for a state that does not own the surface, so
            // asking the volume being exited first and the one being entered
            // second picks the owner without a separate test.
            Real3 from_grad{0, 0, 0};
            if (this->calc_normal_gradient(vgstate_, &from_grad)
                || this->calc_normal_gradient(vgnext_, &from_grad))
            {
                normal_ = from_grad;
            }
            else if (normal_ == Real3{0, 0, 0})
            {
                normal_ = from_next;
            }
        }
    }

    vgstate_ = vgnext_;

    // Whether a safety is worth buying is a property of the volume, so give
    // the one being entered a single probe rather than inheriting a verdict
    // reached somewhere else
    safety_credit_ = 0;

    CELER_ENSURE(this->is_on_boundary());
}

//---------------------------------------------------------------------------//
/*!
 * Move within the current volume.
 *
 * The straight-line distance *must* be less than the distance to the
 * boundary.
 */
CELER_FUNCTION void VecgeomTrackView::move_internal(real_type dist)
{
    CELER_EXPECT(this->has_next_step());
    CELER_EXPECT(dist > 0 && dist <= next_step_);
    CELER_EXPECT(dist != next_step_ || !this->is_next_boundary());

    // Move and update next_step_
    axpy(dist, dir_, &pos_);
    next_step_ -= dist;
    vgstate_.SetBoundaryState(false);

    CELER_ENSURE(!this->is_on_boundary());
}

//---------------------------------------------------------------------------//
/*!
 * Move within the current volume to a nearby point.
 *
 * \warning It's up to the caller to make sure that the position is
 * "nearby" and within the same volume.
 */
CELER_FUNCTION void VecgeomTrackView::move_internal(Real3 const& pos)
{
    pos_ = pos;
    next_step_ = 0;
    vgstate_.SetBoundaryState(false);

    CELER_ENSURE(!this->is_on_boundary());
}

//---------------------------------------------------------------------------//
/*!
 * Change the track's direction.
 *
 * This happens after a scattering event or movement inside a magnetic field.
 * It resets the calculated distance-to-boundary.
 */
CELER_FUNCTION void VecgeomTrackView::set_dir(Real3 const& newdir)
{
    CELER_EXPECT(is_soft_unit_vector(newdir));
    dir_ = newdir;
    next_step_ = 0;
}

//---------------------------------------------------------------------------//
// PRIVATE MEMBER FUNCTIONS
//---------------------------------------------------------------------------//
/*!
 * Whether a next step has been calculated.
 */
CELER_FUNCTION bool VecgeomTrackView::has_next_step() const
{
    return next_step_ != 0;
}

//---------------------------------------------------------------------------//
/*!
 * Whether the calculated next step will take track to next boundary.
 */
CELER_FUNCTION bool VecgeomTrackView::is_next_boundary() const
{
    CELER_EXPECT(this->has_next_step() || this->is_on_boundary());
    return vgnext_.IsOnBoundary();
}

//---------------------------------------------------------------------------//
/*!
 * Get a reference to the current volume.
 */
CELER_FUNCTION auto VecgeomTrackView::physical_volume() const
    -> VgPlacedVol const&
{
    VgPlacedVol const* physvol_ptr = vgstate_.Top();
    CELER_ENSURE(physvol_ptr);
    return *physvol_ptr;
}

//---------------------------------------------------------------------------//
/*!
 * Get a reference to the current volume, or to world volume if outside.
 */
CELER_FUNCTION auto VecgeomTrackView::logical_volume() const -> VgLogVol const&
{
    return *this->physical_volume().GetLogicalVolume();
}

//---------------------------------------------------------------------------//
}  // namespace celeritas
