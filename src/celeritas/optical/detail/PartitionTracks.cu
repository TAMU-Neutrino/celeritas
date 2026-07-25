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
    ObserverPtr<TrackStatus const, MemSpace::device> status_;

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
size_type partition_alive(DeviceRef<CoreStateData> const& state,
                          size_type num_threads)
{
    CELER_EXPECT(num_threads <= state.track_slots.size());
    using SlotT = TrackSlotId::size_type;
    auto slots
        = state.track_slots[AllItems<SlotT, MemSpace::device>{}];
    auto status = state.sim.status[AllItems<TrackStatus, MemSpace::device>{}];

    auto start = device_pointer_cast(
        ObserverPtr<SlotT, MemSpace::device>{slots.data()});
    auto last = thrust::partition(
        thrust_execute_on(state.stream_id),
        start,
        start + num_threads,
        IsNotInactive{ObserverPtr<TrackStatus const, MemSpace::device>{
            status.data()}});
    CELER_DEVICE_API_CALL(PeekAtLastError());

    return static_cast<size_type>(last - start);
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
