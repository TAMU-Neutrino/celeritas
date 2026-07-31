//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/Types.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Macros.hh"
#include "celeritas/Types.hh"

namespace celeritas
{
//---------------------------------------------------------------------------//
// TYPE ALIASES
//---------------------------------------------------------------------------//
namespace optical
{

//! Opaque index of sub-surface interface
using LocalSurfaceId = OpaqueId<struct LocalSurface_, unsigned short int>;

//! Opaque index of sub-surface material region
using LocalPositionId = OpaqueId<struct LocalMat_, unsigned short int>;

}  // namespace optical

using ScintSpectrumId = OpaqueId<struct ScintSpectrumRecord>;

//---------------------------------------------------------------------------//
// ENUMERATIONS
//---------------------------------------------------------------------------//

//! Process used to generate optical photons
enum class GeneratorType
{
    cherenkov,
    scintillation,
    wls,
    wls2,
    size_
};

namespace optical
{

//! Ordering of surface physics boundary crossing models
enum class SurfacePhysicsOrder
{
    roughness,  //!< Sample a facet normal
    reflectivity,  //!< Select physics override
    interaction,  //!< Apply reflection and refraction
    size_
};

//! Traversal direction of a sub-subsurface
enum class LocalDirection : bool
{
    reverse = false,
    forward = true
};

//! Possible reflection moes for UNIFIED reflection model.
enum class ReflectionMode
{
    specular_spike,
    specular_lobe,
    backscatter,
    diffuse_lobe,
    size_ = diffuse_lobe,
};

//! Trivial interaction modes
enum class TrivialInteractionMode
{
    absorb,  //!< absorb on surface
    transmit,  //!< transmit with no change
    backscatter,  //!< back scatter
};

//! Results of a reflectivity substep
enum class ReflectivityAction
{
    transmit,  //!< transmit with no change
    interact,  //!< continue to sample surface interaction
    absorb,  //!< absorb on surface
    size_ = absorb,
};

//! Optical photon wavelength shifting time model
//! \todo replace with OnedDistributionType
enum class WlsDistribution
{
    delta,  //!< Delta function
    exponential,  //!< Exponential decay
    size_
};

//---------------------------------------------------------------------------//
// FREE FUNCTIONS
//---------------------------------------------------------------------------//

char const* to_cstring(GeneratorType);
char const* to_cstring(SurfacePhysicsOrder);
char const* to_cstring(ReflectionMode);
char const* to_cstring(WlsDistribution);

//! Convert sub-surface direction to a sign (+1/-1 for forward/reverse resp.)
CELER_CONSTEXPR_FUNCTION int to_signed_offset(LocalDirection d)
{
    return 2 * static_cast<int>(d) - 1;
}

//---------------------------------------------------------------------------//
// EVENT ENCODING IN THE PRIMARY ID (streaming injection)
//---------------------------------------------------------------------------//
/*!
 * How an offloading application packs an event ordinal into \c PrimaryId.
 *
 * The low \c event_shift bits hold the application's own track id, the rest
 * an event ordinal that wraps at \c event_ring. Wavelength-shifted photons
 * inherit the whole field, so a re-emitted photon still names its event.
 * Only the event census interprets these; everything else treats the primary
 * id as opaque. They are defined here so the interpretation lives in one
 * place rather than being duplicated by each application.
 */
//!@{
inline constexpr unsigned int event_shift = 20;
inline constexpr unsigned int event_ring = 1u << (32 - event_shift);
//!@}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
