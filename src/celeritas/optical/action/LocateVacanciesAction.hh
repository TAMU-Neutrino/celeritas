//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/LocateVacanciesAction.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/data/AuxInterface.hh"
#include "corecel/sys/ActionInterface.hh"

#include "ActionInterface.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Find the vacant track slots at the end of the step.
 *
 * The per-stream selection scratch (device count, CUB temporary storage,
 * fallback flag buffer) lives in aux state, so this action is both a step
 * action and an aux params interface; it holds \c StaticActionData directly
 * because the concrete-action mixins declare \c label() final.
 *
 * \todo Rename?
 */
class LocateVacanciesAction final : public OpticalStepActionInterface,
                                    public AuxParamsInterface
{
  public:
    // Construct with action ID and aux ID
    LocateVacanciesAction(ActionId, AuxId);

    //!@{
    //! \name Action interface
    //! ID of this action
    ActionId action_id() const final { return sad_.action_id(); }
    //! Short label
    std::string_view label() const final { return sad_.label(); }
    //! Description of the action
    std::string_view description() const final { return sad_.description(); }
    //! Dependency ordering of the action
    StepActionOrder order() const final { return StepActionOrder::end; }
    // Execute the action with host data
    void step(CoreParams const&, CoreStateHost&) const final;
    // Execute the action with device data
    void step(CoreParams const&, CoreStateDevice&) const final;
    //!@}

    //!@{
    //! \name Aux params interface
    //! Index of this class instance in its registry
    AuxId aux_id() const final { return aux_id_; }
    // Build the per-stream selection scratch
    UPState create_state(MemSpace, StreamId, size_type) const final;
    //!@}

  private:
    StaticActionData sad_;
    AuxId aux_id_;

    template<MemSpace M>
    void step_impl(CoreState<M>&) const;
};

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
