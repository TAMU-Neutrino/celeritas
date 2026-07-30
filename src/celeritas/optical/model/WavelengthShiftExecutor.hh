//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/model/WavelengthShiftExecutor.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Macros.hh"
#include "corecel/math/Atomics.hh"
#include "celeritas/geo/GeoFwd.hh"
#include "celeritas/optical/CoreTrackView.hh"
#include "celeritas/optical/Interaction.hh"
#include "celeritas/optical/ParticleTrackView.hh"
#include "celeritas/optical/SimTrackView.hh"
#include "celeritas/optical/interactor/WavelengthShiftInteractor.hh"
#include "celeritas/track/CoreStateCounters.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
struct WavelengthShiftExecutor
{
    inline CELER_FUNCTION Interaction operator()(CoreTrackView const&);

    NativeCRef<WavelengthShiftData> data;
    NativeRef<WlsGeneratorStateData> aux_data;
    size_type buffer_size{};
    // Global step counters: num_dist_written flags fresh distributions so
    // the WLS generator action can skip its pipeline when idle
    CoreStateCounters* counters{nullptr};
};

//---------------------------------------------------------------------------//
/*!
 * Sample optical WLS interaction from the current track.
 */
CELER_FUNCTION Interaction WavelengthShiftExecutor::operator()(
    CoreTrackView const& track)
{
    auto particle = track.particle();
    auto sim = track.sim();
    auto mat_id = track.material_record().material_id();
    auto dist_id = id_cast<ItemId<WlsDistributionData>>(
        buffer_size + track.track_slot_id().get());
    auto rng = track.rng();

    WavelengthShiftInteractor interact{
        data, aux_data, particle, sim, track.geometry().pos(), mat_id, dist_id};

    Interaction result = interact(rng);

    // A record with photons to generate was stored: flag it so the WLS
    // generator pipeline runs (a below-threshold write stays invalid and
    // needs no pipeline pass)
    if (aux_data.distributions[dist_id])
    {
        atomic_add(&counters->num_dist_written, size_type{1});
    }

    return result;
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
