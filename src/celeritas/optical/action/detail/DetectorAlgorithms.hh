//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/detail/DetectorAlgorithms.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "corecel/Types.hh"
#include "celeritas/optical/DetectorData.hh"

#include "DetectorHitBuffer.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
template<MemSpace M>
using DetectorStateRef = DetectorStateData<Ownership::reference, M>;

//---------------------------------------------------------------------------//
//! An actual hit has a valid detector
struct IsHit
{
    CELER_FUNCTION bool operator()(DetectorHit const& hit) const
    {
        return static_cast<bool>(hit);
    }
};

//---------------------------------------------------------------------------//
// Stably compact the valid hits into the buffer's device storage
void copy_if_hit(DetectorStateRef<MemSpace::device> const&,
                 DetectorHitBuffer*,
                 size_type num_expected,
                 StreamId);

//---------------------------------------------------------------------------//
// INLINE DEFINITIONS
//---------------------------------------------------------------------------//
#if !CELER_USE_DEVICE
inline void copy_if_hit(DetectorStateRef<MemSpace::device> const&,
                        DetectorHitBuffer*,
                        size_type,
                        StreamId)
{
    CELER_NOT_CONFIGURED("CUDA or HIP");
}
#endif
//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
