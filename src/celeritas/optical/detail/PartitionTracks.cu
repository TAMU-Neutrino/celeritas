//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/PartitionTracks.cu
//---------------------------------------------------------------------------//
#include "PartitionTracks.hh"

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/partition.h>

#include "corecel/Macros.hh"
#include "corecel/data/ObserverPtr.device.hh"
#include "corecel/sys/Stream.hh"
#include "corecel/sys/Thrust.device.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
namespace
{
//---------------------------------------------------------------------------//
struct IsNotInactive
{
    ObserverPtr<TrackStatus const> status_;

    CELER_FUNCTION bool operator()(TrackSlotId::size_type slot) const
    {
        return status_.get()[slot] != TrackStatus::inactive;
    }
};

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
/*!
 * Move the live tracks to the front of the thread-to-slot map.
 */
size_type partition_alive(DeviceRef<CoreStateData> const& state)
{
    auto slots = state.track_slots[
        AllItems<TrackSlotId::size_type, MemSpace::device>{}];
    auto start = device_pointer_cast(slots.data());

    auto* last = thrust::partition(
        thrust_execute_on(state.stream_id),
        start,
        start + slots.size(),
        IsNotInactive{
            ObserverPtr<TrackStatus const>{
                state.sim.status[AllItems<TrackStatus, MemSpace::device>{}]
                    .data()}});
    CELER_DEVICE_API_CALL(PeekAtLastError());

    return static_cast<size_type>(last - start);
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
