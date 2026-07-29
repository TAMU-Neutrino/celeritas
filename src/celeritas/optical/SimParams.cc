//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/SimParams.cc
//---------------------------------------------------------------------------//
#include "SimParams.hh"

#include <cstdlib>

#include "corecel/Assert.hh"

#include "SimData.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Construct with simulation options.
 */
SimParams::SimParams(inp::OpticalTrackingLimits const& inp)
{
    CELER_VALIDATE(inp.steps > 0 && inp.steps <= inp::TrackingLimits::unlimited,
                   << "maximum step limit " << inp.steps
                   << " is out of range (should be in (0, "
                   << inp::TrackingLimits::unlimited << "])");

    HostVal<SimParamsData> host_data;
    host_data.max_steps = inp.steps;
    host_data.max_step_iters = inp.step_iters;
    if (char const* s = std::getenv("CELER_OPTICAL_MAX_STEP_ITERS"))
    {
        // A flush iterates until its longest-lived track finishes, so one
        // pathological photon holds the device at ~0% occupancy for the
        // whole limit. Tightening the limit bounds that damage.
        host_data.max_step_iters = std::atoi(s);
        CELER_VALIDATE(host_data.max_step_iters > 0,
                       << "invalid CELER_OPTICAL_MAX_STEP_ITERS");
    }

    data_ = ParamsDataStore<SimParamsData>{std::move(host_data)};
    CELER_ENSURE(data_);
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
