//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/DetectorAction.cc
//---------------------------------------------------------------------------//
#include "DetectorAction.hh"

#include "corecel/data/CollectionAlgorithms.hh"
#include "corecel/math/Algorithms.hh"

#include "ActionLauncher.hh"
#include "TrackSlotExecutor.hh"

#include "detail/DetectorExecutor.hh"
#include "detail/DetectorHitBuffer.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Construct with action ID, aux ID, and callback.
 */
DetectorAction::DetectorAction(ActionId aid,
                               AuxId aux_id,
                               CallbackFunc const& callback)
    : sad_{aid, "detector", "Score optical detector hits"}
    , aux_id_{aux_id}
    , callback_(callback)
{
    CELER_EXPECT(aux_id_);
    CELER_EXPECT(callback);
}

//---------------------------------------------------------------------------//
/*!
 * Build the per-stream delivery buffers.
 *
 * The track-slot capacity bounds how many hits a pass can score, so the
 * device compaction buffer and the pinned host landing buffer are sized to
 * it once. Host-memspace states deliver straight from their own state
 * buffer, so their aux state stays empty.
 */
auto DetectorAction::create_state(MemSpace m, StreamId id, size_type size) const
    -> UPState
{
    auto buf = std::make_unique<detail::DetectorHitBuffer>();
    if (m == MemSpace::device)
    {
        buf->compact = DeviceVector<DetectorHit>(size, id);
        buf->result = DeviceVector<size_type>(1, id);
        buf->host.resize(size);
    }
    return buf;
}

//---------------------------------------------------------------------------//
/*!
 * Launch the detector action on host.
 *
 * \todo avoid reallocating the temporary storage at every step, or as an
 * optimization just call contiguous chunks of hits.
 */
void DetectorAction::step(CoreParams const& params, CoreStateHost& state) const
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
    // stale hits for the full-buffer filter below to re-deliver. Running
    // over the whole permutation visits each physical slot exactly once
    // and clears the ones without hits.
    launch_action(state.size(), execute);

    // Skip the hit sweep when this pass scored nothing: the cumulative hit
    // counter has not moved since the last delivery
    if (counters->num_hits == state.last_hit_count())
    {
        return;
    }
    state.last_hit_count(counters->num_hits);

    auto all_hits
        = state.ref()
              .detectors.detector_hits[AllItems<DetectorHit, MemSpace::host>{}];

    VecHit temp_hits(all_hits.size());
    // Copy all valid hits, erasing remaining part of the vector
    temp_hits.erase(
        std::copy_if(
            all_hits.begin(), all_hits.end(), temp_hits.begin(), Identity{}),
        temp_hits.end());
    this->callback_hits(make_span(temp_hits), state);
}

//---------------------------------------------------------------------------//
#if !CELER_USE_DEVICE
void DetectorAction::step(CoreParams const&, CoreStateDevice&) const
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
#endif

//---------------------------------------------------------------------------//
/*!
 * Process hits and send them to the callback.
 *
 * Only valid hits arrive here. The callback is only executed when a
 * non-zero number of valid hits occurs. A state with a hit sink (streaming
 * mode) receives the hits there instead: the sink runs on the transport
 * thread and hands them to the producer thread, whose thread-local receiver
 * state the global callback may depend on.
 */
void DetectorAction::callback_hits(Span<DetectorHit const> hits,
                                   CoreStateBase const& state) const
{
    if (hits.empty())
    {
        return;
    }
    if (auto const& sink = state.hit_sink())
    {
        sink(hits);
        return;
    }
    callback_(hits);
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
