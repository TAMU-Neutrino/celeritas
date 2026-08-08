//------------------------------ -*- cuda -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/EventCensus.cu
//---------------------------------------------------------------------------//
#include "EventCensus.hh"

#include "corecel/sys/KernelLauncher.device.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
namespace
{
//---------------------------------------------------------------------------//
struct CensusTracksExecutor
{
    DeviceRef<CoreStateData> state;

    CELER_FUNCTION void operator()(ThreadId tid) const
    {
        // track_slots is a ThreadItems collection, so it hands out a span
        // rather than indexing by ThreadId directly -- the same two-step the
        // host implementation uses. Indexing it with the thread id compiles
        // nowhere, and this file only ever builds with CUDA, which is why it
        // survived until a device build ran.
        auto const slots
            = state.track_slots[AllItems<TrackSlotId::size_type,
                                         MemSpace::device>{}];
        TrackSlotId slot{slots[tid.unchecked_get()]};
        if (!is_track_valid(state.sim.status[slot]))
        {
            return;
        }
        census_event(state.init.counters.data().get(),
                     state.sim.primary_ids[slot]);
    }
};

//---------------------------------------------------------------------------//
struct CensusWlsExecutor
{
    DeviceRef<CoreStateData> state;
    DeviceRef<WlsGeneratorStateData> gen;

    CELER_FUNCTION void operator()(ThreadId tid) const
    {
        auto const& dist = gen.distributions[ItemId<WlsDistributionData>(
            tid.unchecked_get())];
        if (dist)
        {
            census_event(state.init.counters.data().get(), dist.primary);
        }
    }
};

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
/*!
 * Fold the live tracks into the event census.
 */
void census_tracks(DeviceRef<CoreStateData> const& state, size_type num_threads)
{
    CELER_EXPECT(num_threads <= state.track_slots.size());
    CensusTracksExecutor execute{state};
    static KernelLauncher<decltype(execute)> const launch("census-tracks");
    launch(num_threads, state.stream_id, execute);
}

//---------------------------------------------------------------------------//
/*!
 * Fold pending wavelength-shift records into the event census.
 */
void census_wls(DeviceRef<CoreStateData> const& state,
                DeviceRef<WlsGeneratorStateData> const& gen,
                size_type buffer_size)
{
    CensusWlsExecutor execute{state, gen};
    static KernelLauncher<decltype(execute)> const launch("census-wls");
    launch(buffer_size, state.stream_id, execute);
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
