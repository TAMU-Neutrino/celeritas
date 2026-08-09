//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/detail/DetectorHitBuffer.hh
//---------------------------------------------------------------------------//
#pragma once

#include <vector>

#include "corecel/Types.hh"
#include "corecel/data/AuxInterface.hh"
#include "corecel/data/DeviceVector.hh"
#include "corecel/data/PinnedAllocator.hh"
#include "celeritas/optical/DetectorData.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Per-stream persistent buffers for delivering detector hits.
 *
 * The device vectors hold the stable compaction of the valid hits, the
 * one-element selected count, and the CUB temporary storage; the pinned
 * host vector is the landing area for the device-to-host copy of exactly
 * the compacted hits. The track-slot capacity bounds how many hits one
 * pass can score, so everything is sized once at state creation (the CUB
 * scratch on first use) and steady-state deliveries allocate nothing.
 *
 * Host-memspace states leave every member empty: the host step delivers
 * straight from its own buffer and never touches this.
 */
struct DetectorHitBuffer final : public AuxStateInterface
{
    DeviceVector<DetectorHit> compact;
    DeviceVector<size_type> result;
    DeviceVector<char> temp;
    std::vector<DetectorHit, PinnedAllocator<DetectorHit>> host;
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
