//------------------------------ -*- cuda -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/DetectorAction.cu
//---------------------------------------------------------------------------//
#include "DetectorAction.hh"

#include "corecel/data/AuxStateVec.hh"
#include "corecel/data/Copier.hh"

#include "ActionLauncher.device.hh"
#include "TrackSlotExecutor.hh"

#include "detail/DetectorAlgorithms.hh"
#include "detail/DetectorExecutor.hh"
#include "detail/DetectorHitBuffer.hh"

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

    // One counter copy (and its sync) decides whether any hits need to come
    // over: most tail iterations score nothing, and skipping them saves the
    // whole delivery
    auto snapshot = state.sync_get_counters();
    if (snapshot.num_hits == state.last_hit_count())
    {
        return;
    }
    // The counter delta IS the number of valid hits this pass scored: every
    // valid hit incremented it in the same kernel that wrote the hit, and
    // the snapshot above synchronized the stream
    size_type const num_new = snapshot.num_hits - state.last_hit_count();
    state.last_hit_count(snapshot.num_hits);

    // Compact the valid hits on the device and bring over exactly that many
    // into pinned memory, instead of copying the whole per-slot buffer into
    // a freshly allocated pageable vector and filtering it on the CPU
    auto& buf
        = get<detail::DetectorHitBuffer>(*state.aux(), this->aux_id());
    detail::copy_if_hit(
        state.ref().detectors, &buf, num_new, state.stream_id());
    CELER_ASSERT(num_new <= buf.host.size());
    Copier<DetectorHit, MemSpace::host> copy_hits{
        {buf.host.data(), num_new}, state.stream_id()};
    copy_hits(MemSpace::device, {buf.compact.data(), num_new});
    celeritas::device().stream(state.stream_id()).sync();

    this->callback_hits({buf.host.data(), num_new}, state);
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
