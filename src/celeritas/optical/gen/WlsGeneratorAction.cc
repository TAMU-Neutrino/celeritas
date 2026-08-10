//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/gen/WlsGeneratorAction.cc
//---------------------------------------------------------------------------//
#include "WlsGeneratorAction.hh"

#include <algorithm>

#include "corecel/Assert.hh"
#include "corecel/data/AuxParamsRegistry.hh"
#include "corecel/data/AuxStateVec.hh"
#include "corecel/io/Logger.hh"
#include "corecel/sys/ActionRegistry.hh"
#include "corecel/sys/KernelLauncher.hh"
#include "celeritas/optical/CoreParams.hh"
#include "celeritas/optical/CoreState.hh"
#include "celeritas/optical/CoreTrackData.hh"
#include "celeritas/optical/WavelengthShiftData.hh"
#include "celeritas/optical/action/ActionLauncher.hh"
#include "celeritas/optical/model/WavelengthShiftModel.hh"
#include "celeritas/phys/GeneratorRegistry.hh"

#include "WavelengthShiftGenerator.hh"

#include "detail/GeneratorAlgorithms.hh"
#include "detail/OffloadAlgorithms.hh"
#include "detail/WlsGeneratorExecutor.hh"
#include "celeritas/optical/detail/EventCensus.hh"

namespace celeritas
{
namespace optical
{
namespace
{
//---------------------------------------------------------------------------//
//! Construct a state
template<MemSpace M>
auto make_state(StreamId stream, size_type size)
{
    using StoreT = StateDataStore<WlsGeneratorStateData, M>;

    auto result = std::make_unique<WlsGeneratorState<M>>();
    result->store = StoreT{stream, size};

    CELER_ENSURE(*result);
    return result;
}

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
/*!
 * Construct with IDs, WLS model, and buffer capacity.
 *
 * The second WLS model is optional.
 */
WlsGeneratorAction::WlsGeneratorAction(Input&& input)
    : GeneratorBase(input.action_id,
                    input.aux_id,
                    input.gen_id,
                    "wls-generate",
                    "generate photons from a wavelength shifting process")
    , wls_(std::move(input.wls))
    , wls2_(std::move(input.wls2))
    , capacity_(input.capacity)
    , census_period_(input.census_period)
{
    CELER_EXPECT(capacity_ > 0);
    CELER_EXPECT(wls_ || wls2_);
}

//---------------------------------------------------------------------------//
/*!
 * Build state data for a stream.
 */
auto WlsGeneratorAction::create_state(MemSpace m, StreamId id, size_type) const
    -> UPState
{
    if (m == MemSpace::host)
    {
        return make_state<MemSpace::host>(id, capacity_);
    }
    else if (m == MemSpace::device)
    {
        return make_state<MemSpace::device>(id, capacity_);
    }
    CELER_ASSERT_UNREACHABLE();
}

//---------------------------------------------------------------------------//
/*!
 * Execute the action with host data.
 */
void WlsGeneratorAction::step(CoreParams const& params,
                              CoreStateHost& state) const
{
    this->step_impl(params, state);
}

//---------------------------------------------------------------------------//
/*!
 * Execute the action with device data.
 */
void WlsGeneratorAction::step(CoreParams const& params,
                              CoreStateDevice& state) const
{
    this->step_impl(params, state);
}

//---------------------------------------------------------------------------//
/*!
 * Generate optical WLS photons from distribution data.
 *
 * \todo Accumulate the total number of steps (distributions) in \c
 * accum.buffer_size
 */
template<MemSpace M>
void WlsGeneratorAction::step_impl(CoreParams const& params,
                                   CoreState<M>& state) const
{
    CELER_EXPECT(state.aux());

    using DistId = ItemId<WlsDistributionData>;
    using DistRange = ItemRange<WlsDistributionData>;

    auto& aux_state = get<WlsGeneratorState<M>>(*state.aux(), this->aux_id());
    auto& counters = aux_state.counters;
    auto& buffer = aux_state.store.ref().distributions;

    // One synchronized read serves the whole pass
    auto core_counters = state.sync_get_counters();

    if (counters.buffer_size == 0 && core_counters.num_dist_written == 0)
    {
        // Nothing is buffered and no track has stored a distribution since
        // the last pass: the compact, scan, fill, and counter update would
        // all be no-ops, so skip their launches and synchronizations. Most
        // tail iterations take this path. (The primary generator action
        // maintains num_active every iteration regardless.)
        return;
    }

    auto num_pending_prev = counters.num_pending;

    // Compact the buffer, returning the total number of valid distributions
    counters.buffer_size = celeritas::detail::remove_if_invalid(
        buffer,
        0,
        counters.buffer_size + state.size(),
        &aux_state.scratch,
        state.stream_id());

    if (counters.buffer_size > 0)
    {
        // If this process created photons, calculate the cumulative sum of the
        // number of photons in the buffered distributions. This is used to
        // determine which thread will generate photons from which distribution
        counters.num_pending = detail::inclusive_scan_photons(
            aux_state.store.ref().distributions,
            aux_state.store.ref().offsets,
            counters.buffer_size,
            &aux_state.scratch,
            state.stream_id());
    }

    // Update the core state counters with the number of new pending tracks,
    // consuming the fresh-distribution flag in the same write. Tracks store
    // distributions in later (post-step) actions of the iteration, so
    // nothing new can arrive between the read above and this write.
    core_counters.num_pending += counters.num_pending - num_pending_prev;
    core_counters.num_dist_written = 0;
    state.sync_put_counters(core_counters);

    if (counters.num_pending > 0 && core_counters.num_vacancies > 0)
    {
        // Generate the optical photons from the distribution data
        this->generate(params, state, core_counters.num_vacancies);

        // Compact the buffer again to remove stale distributions and free up
        // space to add new distributions during this step
        counters.buffer_size = celeritas::detail::remove_if_invalid(
            buffer, 0, counters.buffer_size, &aux_state.scratch, state.stream_id());
    }

    // Ensure the buffer is large enough to hold WLS distributions created
    // during this step
    CELER_VALIDATE(counters.buffer_size + state.size() <= buffer.size(),
                   << "insufficient capacity (" << buffer.size()
                   << ") for buffered optical WLS distribution data (total "
                      "capacity requirement of "
                   << counters.buffer_size + state.size() << ")");

    // Clear data that tracks might write distributions to in this step, which
    // is in an unspecified state after calling \c remove_if on the buffer
    Filler<WlsDistributionData, M> fill{{}, state.stream_id()};
    fill(buffer[DistRange(DistId(counters.buffer_size),
                          DistId(counters.buffer_size + state.size()))]);

    // Fold the records still waiting to be re-emitted into the event census,
    // so an event with a queued wavelength-shifted photon is never mistaken
    // for finished and released while its light is still coming. Contributing
    // on a non-census iteration is harmless: the value is reset at the start
    // of each census, so only contributions inside the window are read.
    if (this->census_enabled() && counters.buffer_size > 0)
    {
        this->census(state, counters.buffer_size);
    }

    // Update the generator and optical core state counters (the kernels
    // above do not touch them, so the snapshot is still current)
    this->update_counters(state, core_counters);

    CELER_ENSURE(!counters.buffer_size == !counters.num_pending);
}

//---------------------------------------------------------------------------//
/*!
 * Fold pending re-emission records into the event census (host).
 */
void WlsGeneratorAction::census(CoreStateHost& state,
                                size_type buffer_size) const
{
    CELER_EXPECT(state.aux());

    auto& aux_state = get<WlsGeneratorState<MemSpace::native>>(*state.aux(),
                                                               this->aux_id());
    detail::census_wls(state.ref(), aux_state.store.ref(), buffer_size);
}

//---------------------------------------------------------------------------//
#if !CELER_USE_DEVICE
void WlsGeneratorAction::census(CoreStateDevice&, size_type) const
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
#endif

//---------------------------------------------------------------------------//
/*!
 * Launch a (host) kernel to generate optical photons.
 */
void WlsGeneratorAction::generate(CoreParams const& params,
                                  CoreStateHost& state,
                                  size_type num_vacancies) const
{
    CELER_EXPECT(state.aux());

    auto& aux_state = get<WlsGeneratorState<MemSpace::native>>(*state.aux(),
                                                               this->aux_id());
    // num_vacancies comes from the caller's read: another one here would
    // cost a stream synchronization for a value that cannot have changed
    size_type num_gen = min(num_vacancies, aux_state.counters.num_pending);

    // Generate optical photons in vacant track slots
    detail::WlsGeneratorExecutor execute{
        params.ptr<MemSpace::native>(),
        state.ptr(),
        wls_ ? wls_->host_ref() : NativeCRef<WavelengthShiftData>{},
        wls2_ ? wls2_->host_ref() : NativeCRef<WavelengthShiftData>{},
        aux_state.store.ref(),
        aux_state.counters.buffer_size};
    launch_action(num_gen, execute);
}

//---------------------------------------------------------------------------//
#if !CELER_USE_DEVICE
void WlsGeneratorAction::generate(CoreParams const&, CoreStateDevice&, size_type) const
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
#endif

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
