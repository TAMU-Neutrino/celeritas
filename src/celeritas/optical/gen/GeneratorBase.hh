//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/gen/GeneratorBase.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Macros.hh"
#include "corecel/data/AuxInterface.hh"
#include "corecel/data/AuxStateVec.hh"
#include "corecel/math/Algorithms.hh"
#include "corecel/sys/ActionInterface.hh"
#include "celeritas/optical/action/ActionInterface.hh"
#include "celeritas/phys/GeneratorInterface.hh"
#include "celeritas/track/CoreStateCounters.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Common interface for generating optical photons.
 */
class GeneratorBase : virtual public optical::OpticalStepActionInterface,
                      virtual public AuxParamsInterface,
                      virtual public GeneratorInterface
{
  public:
    // Construct from IDs, unique label, and description
    GeneratorBase(ActionId id,
                  AuxId aux_id,
                  GeneratorId gen_id,
                  std::string_view label,
                  std::string_view description) noexcept(!CELERITAS_DEBUG);

    //!@{
    //! \name Aux interface

    //! Index of this class instance in its registry
    AuxId aux_id() const final { return aux_id_; }
    //!@}

    //!@{
    //! \name Action interface

    //! ID of the action
    ActionId action_id() const final { return sad_.action_id(); }
    //! Short name for the action
    std::string_view label() const final { return sad_.label(); }
    //! Description of the action
    std::string_view description() const final { return sad_.description(); }
    //!@}

    //!@{
    //! \name StepAction interface

    //! Dependency ordering of the action
    StepActionOrder order() const final { return StepActionOrder::generate; }
    //!@}

    //!@{
    //! \name Generator interface

    //! ID of the generator
    GeneratorId generator_id() const final { return gen_id_; }
    // Get generator counters (mutable)
    GeneratorStateBase& counters(AuxStateVec&) const final;
    // Get generator counters
    GeneratorStateBase const& counters(AuxStateVec const&) const final;
    //!@}

    // Update the generator and state counters
    template<MemSpace M>
    inline void update_counters(optical::CoreState<M>&) const;

    // Same, reusing a counter snapshot the caller already synchronized:
    // saves the second device round trip per generator per iteration
    template<MemSpace M>
    inline void
    update_counters(optical::CoreState<M>&, CoreStateCounters&) const;

  private:
    StaticActionData sad_;
    AuxId aux_id_;
    GeneratorId gen_id_;
};

//---------------------------------------------------------------------------//
// INLINE DEFINITIONS
//---------------------------------------------------------------------------//
/*!
 * Update the generator and state counters.
 */
template<MemSpace M>
void GeneratorBase::update_counters(optical::CoreState<M>& state) const
{
    auto counters = state.sync_get_counters();
    this->update_counters(state, counters);
}

//---------------------------------------------------------------------------//
/*!
 * Update the generator and state counters from a synchronized snapshot.
 *
 * The generator kernels do not modify the core state counters, so a snapshot
 * read at the top of the generator's step is still current here and the
 * device round trip of a second read can be skipped.
 */
template<MemSpace M>
void GeneratorBase::update_counters(optical::CoreState<M>& state,
                                    CoreStateCounters& counters) const
{
    CELER_EXPECT(state.aux());

    auto& gen_counters = this->counters(*state.aux());

    // Calculate the number of new tracks generated at this step
    size_type num_gen
        = min(counters.num_vacancies, gen_counters.counters.num_pending);

    // Update the optical core state counters
    counters.num_pending -= num_gen;
    counters.num_generated += num_gen;
    counters.num_vacancies -= num_gen;

    // Update the generator counters and statistics
    gen_counters.counters.num_pending -= num_gen;
    gen_counters.counters.num_generated += num_gen;
    gen_counters.accum.num_generated += num_gen;

    // Update the number of active tracks. This must be done even if no new
    // tracks were generated
    counters.num_active = state.size() - counters.num_vacancies;
    state.sync_put_counters(counters);
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
