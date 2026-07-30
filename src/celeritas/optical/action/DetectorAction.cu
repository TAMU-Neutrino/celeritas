//------------------------------ -*- cuda -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/DetectorAction.cu
//---------------------------------------------------------------------------//
#include "DetectorAction.hh"

#include "ActionLauncher.device.hh"
#include "TrackSlotExecutor.hh"

#include "detail/DetectorExecutor.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Launch the detector action on device.
 */
void DetectorAction::step(CoreParams const& params, CoreStateDevice& state) const
{
    auto* counters
        = static_cast<CoreStateCounters*>(state.ref().init.counters.data());
    TrackSlotExecutor execute{
        params.ptr<MemSpace::native>(),
        state.ptr(),
        detail::DetectorExecutor{state.ref().detectors, counters}};

    static ActionLauncher<decltype(execute)> const launch_kernel(*this);
    launch_kernel(state, execute);

    // One counter copy (and its sync) decides whether the full hit buffer
    // needs to come over: most tail iterations score nothing, and skipping
    // them saves the buffer copy and a second synchronization
    auto snapshot = state.sync_get_counters();
    if (snapshot.num_hits == state.last_hit_count())
    {
        return;
    }
    state.last_hit_count(snapshot.num_hits);

    this->callback_hits(this->load_hits_sync(state));
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
