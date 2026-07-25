//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/detail/PartitionTracks.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Types.hh"
#include "celeritas/optical/CoreTrackData.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
// Move the live tracks to the front of the first num_threads entries of the
// thread-to-slot map
size_type partition_alive(HostRef<CoreStateData> const&, size_type num_threads);
size_type
partition_alive(DeviceRef<CoreStateData> const&, size_type num_threads);

#if !CELER_USE_DEVICE
inline size_type partition_alive(DeviceRef<CoreStateData> const&, size_type)
{
    CELER_NOT_CONFIGURED("CUDA OR HIP");
}
#endif

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
