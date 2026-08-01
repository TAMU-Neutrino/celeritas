//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/Transporter.hh
//---------------------------------------------------------------------------//
#pragma once

#include <memory>

#include "corecel/Types.hh"
#include "corecel/sys/ActionGroups.hh"
#include "celeritas/Types.hh"
#include "celeritas/user/ActionTimes.hh"
#include "celeritas/user/StepTimes.hh"

#include "CoreState.hh"

namespace celeritas
{
template<class P, template<MemSpace M> class S>
class ActionGroups;

namespace optical
{
class CoreParams;
template<MemSpace M>
class CoreState;
class CoreStateBase;

//---------------------------------------------------------------------------//
/*!
 * Transport all pending optical tracks to completion.
 *
 * \note This class must be constructed \em after all optical actions have been
 * added to the action registry.
 */
class Transporter
{
  public:
    //!@{
    //! \name Type aliases
    using CoreStateHost = CoreState<MemSpace::host>;
    using CoreStateDevice = CoreState<MemSpace::device>;
    using SPConstParams = std::shared_ptr<CoreParams const>;
    using SPActionTimes = std::shared_ptr<ActionTimes>;
    using SPStepTimes = std::shared_ptr<StepTimes>;
    using MapStrDbl = ActionTimes::MapStrDbl;
    using VecDbl = StepTimes::VecDbl;
    //!@}

    struct Input
    {
        SPConstParams params;
        SPActionTimes action_times;  //!< Optional
        SPStepTimes step_times;  //!< Optional
        //! Iterations between event censuses; 0 disables
        size_type census_period{0};
    };

  public:
    // Construct with problem parameters and setup options
    explicit Transporter(Input&&);

    // Transport all pending optical tracks
    void operator()(CoreStateBase&) const;

    // Run a single step iteration: the loop body of operator(), exposed so
    // the streaming driver can interleave stepping with injection. The
    // ordinal sequences the periodic full compaction pass and must increase
    // by one per call on a given state. Returns the counters synchronized
    // at the end of the iteration.
    // census_base is the event ordinal the census reduction is relative to;
    // a streaming driver moves it forward as it retires events, keeping
    // every ordinal in flight within one ring of it.
    CoreStateCounters step_once(CoreStateBase&,
                                size_type iter_ordinal,
                                size_type census_base = 0) const;

    //! Access the shared params
    SPConstParams const& params() const { return input_.params; }

    // Get the accumulated action times
    MapStrDbl get_action_times(AuxStateVec const&) const;

    // Get the recorded step times
    VecDbl get_step_times(AuxStateVec const&) const;

  private:
    //// TYPES ////

    using ActionGroupsT = ActionGroups<CoreParams, CoreState>;
    using SPActionGroups = std::shared_ptr<ActionGroupsT>;

    //// DATA ////

    Input input_;
    SPActionGroups actions_;

  public:
    //! Whether the event census is running
    bool census_enabled() const { return input_.census_period > 0; }

  private:

    //// HELPERS ////

    template<MemSpace M>
    void transport_impl(CoreState<M>&) const;

    template<MemSpace M>
    CoreStateCounters step_once_impl(CoreState<M>&,
                                     size_type iter_ordinal,
                                     size_type census_base) const;
};

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
