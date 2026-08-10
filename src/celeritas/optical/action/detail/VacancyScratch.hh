//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/detail/VacancyScratch.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Types.hh"
#include "corecel/data/AuxInterface.hh"
#include "corecel/data/DeviceVector.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Per-stream persistent scratch for the vacancy selection.
 *
 * The vacancy pass runs every loop iteration, and each device call used to
 * allocate and free a one-element selected count, the CUB temporary storage,
 * and (on the transform-plus-Flagged fallback) a per-slot flag buffer --
 * several stream-ordered pool operations per iteration for buffers whose
 * sizes never change: they are all functions of the fixed track-slot
 * capacity. Holding them here, the count is sized at state creation and the
 * CUB scratch and flag buffer are grown once on first use, so steady-state
 * iterations allocate nothing.
 *
 * Host-memspace states leave every member empty: the host selection is a
 * plain loop and never touches this.
 */
struct VacancyScratch final : public AuxStateInterface
{
    DeviceVector<size_type> result;
    DeviceVector<char> temp;
    DeviceVector<unsigned char> flags;
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
