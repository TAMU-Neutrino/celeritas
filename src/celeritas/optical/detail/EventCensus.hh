//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/EventCensus.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "corecel/Types.hh"
#include "corecel/math/Atomics.hh"
#include "celeritas/optical/CoreTrackData.hh"
#include "celeritas/optical/Types.hh"
#include "celeritas/optical/WavelengthShiftData.hh"
#include "celeritas/track/CoreStateCounters.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Reduce the smallest live event ordinal, relative to a host-chosen base.
 *
 * An offloading application packs its event ordinal into the high bits of the
 * primary id (the low bits being its own track id); wavelength-shifted
 * photons inherit the field, so a re-emitted photon still names its event.
 *
 * The reduction is relative and modular because that field wraps: differences
 * from the base order correctly as long as fewer events are in flight than
 * the ring holds, which a streaming driver's backpressure guarantees. The
 * caller clears the target to \c event_ring first, so "nothing live" reads
 * back as the ring size.
 */
inline CELER_FUNCTION void
census_event(CoreStateCounters* counters, PrimaryId primary)
{
    size_type const ev
        = primary ? (primary.unchecked_get() >> event_shift) : 0;
    size_type const rel = (ev - counters->event_census_base) & (event_ring - 1);
    atomic_min(&counters->min_live_event_rel, rel);
}

//---------------------------------------------------------------------------//
// Fold the live tracks into the event census
void census_tracks(HostRef<CoreStateData> const&, size_type num_threads);
void census_tracks(DeviceRef<CoreStateData> const&, size_type num_threads);

// Fold pending wavelength-shift records into the event census
void census_wls(HostRef<CoreStateData> const&,
                HostRef<WlsGeneratorStateData> const&,
                size_type buffer_size);
void census_wls(DeviceRef<CoreStateData> const&,
                DeviceRef<WlsGeneratorStateData> const&,
                size_type buffer_size);

#if !CELER_USE_DEVICE
inline void census_tracks(DeviceRef<CoreStateData> const&, size_type)
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
inline void census_wls(DeviceRef<CoreStateData> const&,
                       DeviceRef<WlsGeneratorStateData> const&,
                       size_type)
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
#endif

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
