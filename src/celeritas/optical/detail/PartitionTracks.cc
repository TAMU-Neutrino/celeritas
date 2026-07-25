//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/PartitionTracks.cc
//---------------------------------------------------------------------------//
#include "PartitionTracks.hh"

#include <algorithm>

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Move the live tracks to the front of the thread-to-slot map.
 *
 * Returns the number of live tracks, which is how many threads a launch has
 * to cover. The map stays a permutation of every slot, so the empty ones are
 * still reachable by the actions that need them.
 */
size_type partition_alive(HostRef<CoreStateData> const& state)
{
    auto slots
        = state.track_slots[AllItems<TrackSlotId::size_type, MemSpace::host>{}];
    auto status = state.sim.status[AllItems<TrackStatus, MemSpace::host>{}];

    auto* last = std::partition(
        slots.begin(), slots.end(), [&status](TrackSlotId::size_type slot) {
            return status[slot] != TrackStatus::inactive;
        });
    return static_cast<size_type>(last - slots.begin());
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
