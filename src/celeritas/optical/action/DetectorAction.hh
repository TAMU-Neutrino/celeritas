//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/DetectorAction.hh
//---------------------------------------------------------------------------//
#pragma once

#include <algorithm>

#include "corecel/cont/Span.hh"
#include "corecel/data/AuxInterface.hh"
#include "corecel/sys/ActionInterface.hh"
#include "celeritas/inp/Scoring.hh"
#include "celeritas/optical/CoreParams.hh"
#include "celeritas/optical/CoreState.hh"
#include "celeritas/optical/DetectorData.hh"

#include "ActionInterface.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Record sensitive detector data for optical photons at the end of every step.
 *
 * The \c DetectorExecutor is responsible for copying hit data for every photon
 * into the state buffer at the end of every step on a kernel level. Even if a
 * track was not in a detector, it is still copied into the state buffer with
 * an invalid detector ID. On device the valid hits are compacted in place on
 * the GPU and exactly that many come to pinned host memory; on host the
 * buffer is filtered directly. A span of only valid hits is then passed into
 * the user provided callback function.
 *
 * The per-stream delivery buffers live in aux state, so this action is both
 * a step action and an aux params interface; it holds \c StaticActionData
 * directly because the concrete-action mixins declare \c label() final.
 */
class DetectorAction final : public OpticalStepActionInterface,
                             public AuxParamsInterface
{
  public:
    //!@{
    //! \name Type aliases
    using CallbackFunc = inp::OpticalDetector::HitCallbackFunc;
    //!@}

  public:
    // Construct with action ID, aux ID, and callback function
    DetectorAction(ActionId, AuxId, CallbackFunc const&);

    //!@{
    //! \name Action interface
    //! ID of this action
    ActionId action_id() const final { return sad_.action_id(); }
    //! Short label
    std::string_view label() const final { return sad_.label(); }
    //! Description of the action
    std::string_view description() const final { return sad_.description(); }
    //! Dependency ordering of the action
    StepActionOrder order() const final { return StepActionOrder::post; }
    // Launch kernel with host data
    void step(CoreParams const&, CoreStateHost&) const final;
    // Launch kernel with device data
    void step(CoreParams const&, CoreStateDevice&) const final;
    //!@}

    //!@{
    //! \name Aux params interface
    //! Index of this class instance in its registry
    AuxId aux_id() const final { return aux_id_; }
    // Build the per-stream delivery buffers
    UPState create_state(MemSpace, StreamId, size_type) const final;
    //!@}

  private:
    //// TYPES ////

    using VecHit = std::vector<DetectorHit>;

    //// DATA ////

    StaticActionData sad_;
    AuxId aux_id_;
    CallbackFunc callback_;

    //// HELPER FUNCTIONS ////

    // Send hits to the state's sink if set, else the global callback
    void callback_hits(Span<DetectorHit const>, CoreStateBase const&) const;
};

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
