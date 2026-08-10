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

    // EVERY track slot, not the compacted active prefix: the executor
    // writes hits through the track-slot indirection, so a compacted
    // launch's physical slots are scattered and slots it skips would keep
    // stale hits for the full-range selection below to re-deliver. Running
    // over the whole permutation visits each physical slot exactly once and
    // clears the ones without hits, which is the executor's own documented
    // contract (and the launcher's comment points compaction-exempt actions
    // at exactly this overload).
    static ActionLauncher<decltype(execute)> const launch_kernel(*this);
    launch_kernel(range(ThreadId{state.size()}), state.stream_id(), execute);

    // One counter copy (and its sync) decides whether the full hit buffer
    // needs to come over: most tail iterations score nothing, and skipping
    // them saves the buffer copy and a second synchronization
    auto snapshot = state.sync_get_counters();
    if (snapshot.num_hits == state.last_hit_count())
    {
        return;
    }
    state.last_hit_count(snapshot.num_hits);

    this->callback_hits(this->load_hits_sync(state), state);
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
