//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/SortTracks.cc
//---------------------------------------------------------------------------//
#include "SortTracks.hh"

#include <algorithm>

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Order the thread-to-slot map by volume on the host.
 */
size_type sort_by_volume(HostRef<CoreStateData> const& state,
                         size_type num_threads)
{
    CELER_EXPECT(num_threads <= state.track_slots.size());
#if CELERITAS_CORE_GEO == CELERITAS_CORE_GEO_VECGEOM \
    && CELER_VGNAV != CELER_VGNAV_PATH
    using SlotT = TrackSlotId::size_type;
    auto slots = state.track_slots[AllItems<SlotT, MemSpace::host>{}];
    auto nav = state.geometry.state[AllItems<VgNavStateImpl, MemSpace::host>{}];
    std::sort(slots.data(),
              slots.data() + num_threads,
              [&nav](SlotT a, SlotT b) { return nav[a] < nav[b]; });
#else
    (void)state;
#endif
    return num_threads;
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
