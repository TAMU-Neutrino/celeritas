//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/LocateVacanciesAction.cc
//---------------------------------------------------------------------------//
#include "LocateVacanciesAction.hh"

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "corecel/data/AuxStateVec.hh"
#include "celeritas/optical/CoreParams.hh"
#include "celeritas/optical/CoreState.hh"

#include "detail/TrackInitAlgorithms.hh"
#include "detail/VacancyScratch.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Construct with action ID and aux ID.
 */
LocateVacanciesAction::LocateVacanciesAction(ActionId aid, AuxId aux_id)
    : sad_{aid, "locate-vacancies", "locate vacant track states"}
    , aux_id_{aux_id}
{
    CELER_EXPECT(aux_id_);
}

//---------------------------------------------------------------------------//
/*!
 * Build the per-stream selection scratch.
 *
 * Only the one-element selected count is sized here: the CUB temporary
 * storage and the fallback flag buffer are grown on the first selection,
 * where the device algorithm choice is known, and their sizes depend only
 * on the fixed track-slot capacity so the first call's growth serves every
 * later one. Host-memspace states never touch the scratch, so theirs stays
 * empty.
 */
auto LocateVacanciesAction::create_state(MemSpace m, StreamId id, size_type) const
    -> UPState
{
    auto scratch = std::make_unique<detail::VacancyScratch>();
    if (m == MemSpace::device)
    {
        scratch->result = DeviceVector<size_type>(1, id);
    }
    return scratch;
}

//---------------------------------------------------------------------------//
/*!
 * Execute the action with host data.
 */
void LocateVacanciesAction::step(CoreParams const&, CoreStateHost& state) const
{
    return this->step_impl(state);
}

//---------------------------------------------------------------------------//
/*!
 * Execute the action with device data.
 */
void LocateVacanciesAction::step(CoreParams const&, CoreStateDevice& state) const
{
    return this->step_impl(state);
}

//---------------------------------------------------------------------------//
/*!
 * Initialize optical track states.
 */
template<MemSpace M>
void LocateVacanciesAction::step_impl(CoreState<M>& state) const
{
    auto counters = state.sync_get_counters();
    auto& scratch
        = get<detail::VacancyScratch>(*state.aux(), this->aux_id());

    // Compact the IDs of the inactive tracks, getting the sorted indices of
    // the empty slots
    counters.num_vacancies = detail::copy_if_vacant(state.ref().sim.status,
                                                    state.ref().init.vacancies,
                                                    &scratch,
                                                    state.stream_id());

    counters.num_alive = state.size() - counters.num_vacancies;
    state.sync_put_counters(counters);
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
