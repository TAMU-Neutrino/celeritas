//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/EventCensus.cc
//---------------------------------------------------------------------------//
#include "EventCensus.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Fold the live tracks into the event census.
 */
void census_tracks(HostRef<CoreStateData> const& state, size_type num_threads)
{
    CELER_EXPECT(num_threads <= state.track_slots.size());
    auto* counters = static_cast<CoreStateCounters*>(state.init.counters.data());
    auto slots
        = state.track_slots[AllItems<TrackSlotId::size_type, MemSpace::host>{}];

    for (size_type i = 0; i < num_threads; ++i)
    {
        TrackSlotId slot{slots[i]};
        if (!is_track_valid(state.sim.status[slot]))
        {
            continue;
        }
        census_event(counters, state.sim.primary_ids[slot]);
    }
}

//---------------------------------------------------------------------------//
/*!
 * Fold pending wavelength-shift records into the event census.
 */
void census_wls(HostRef<CoreStateData> const& state,
                HostRef<WlsGeneratorStateData> const& gen,
                size_type buffer_size)
{
    auto* counters = static_cast<CoreStateCounters*>(state.init.counters.data());
    for (size_type i = 0; i < buffer_size; ++i)
    {
        auto const& dist = gen.distributions[ItemId<WlsDistributionData>(i)];
        if (dist)
        {
            census_event(counters, dist.primary);
        }
    }
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
