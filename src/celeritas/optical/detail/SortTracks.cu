//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/SortTracks.cu
//---------------------------------------------------------------------------//
#include "SortTracks.hh"

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/sort.h>

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
#if CELERITAS_CORE_GEO == CELERITAS_CORE_GEO_VECGEOM \
    && CELER_VGNAV != CELER_VGNAV_PATH
//! Order by the raw navigation index, which identifies the volume path
struct NavIndexLess
{
    ObserverPtr<VgNavStateImpl const, MemSpace::device> nav_;

    CELER_FUNCTION bool
    operator()(TrackSlotId::size_type a, TrackSlotId::size_type b) const
    {
        return nav_.get()[a] < nav_.get()[b];
    }
};
#endif

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
/*!
 * Order the thread-to-slot map by volume.
 */
size_type sort_by_volume(DeviceRef<CoreStateData> const& state,
                         size_type num_threads)
{
    CELER_EXPECT(num_threads <= state.track_slots.size());
#if CELERITAS_CORE_GEO == CELERITAS_CORE_GEO_VECGEOM \
    && CELER_VGNAV != CELER_VGNAV_PATH
    using SlotT = TrackSlotId::size_type;
    auto slots = state.track_slots[AllItems<SlotT, MemSpace::device>{}];
    auto nav
        = state.geometry.state[AllItems<VgNavStateImpl, MemSpace::device>{}];

    auto start = device_pointer_cast(
        ObserverPtr<SlotT, MemSpace::device>{slots.data()});
    thrust::sort(thrust_execute_on(state.stream_id),
                 start,
                 start + num_threads,
                 NavIndexLess{ObserverPtr<VgNavStateImpl const,
                                          MemSpace::device>{nav.data()}});
    CELER_DEVICE_API_CALL(PeekAtLastError());
#else
    (void)state;
#endif
    return num_threads;
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
