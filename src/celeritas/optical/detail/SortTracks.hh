//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/SortTracks.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Types.hh"
#include "celeritas/optical/CoreTrackData.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Order the first num_threads entries of the thread-to-slot map by volume.
 *
 * Tracks in the same volume take the same path through the solid tree, so
 * grouping them makes a warp's lanes agree on where they are going instead of
 * serialising over a handful of different volumes each.
 *
 * This is a TRADE, not a free win: the map is what makes thread i read slot i,
 * so every per-track state array is currently read coalesced, and permuting it
 * scatters those reads. Whether coherence beats coalescing is a property of the
 * geometry and has to be measured.
 */
size_type sort_by_volume(HostRef<CoreStateData> const&, size_type num_threads);
size_type sort_by_volume(DeviceRef<CoreStateData> const&, size_type num_threads);

#if !CELER_USE_DEVICE
inline size_type sort_by_volume(DeviceRef<CoreStateData> const&, size_type)
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
#endif

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
