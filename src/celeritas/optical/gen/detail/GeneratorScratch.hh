//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/gen/detail/GeneratorScratch.hh
//---------------------------------------------------------------------------//
#pragma once

#include <cstddef>

#include "corecel/data/DeviceVector.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Persistent per-stream arena for the generator algorithms' temporaries.
 *
 * The thrust scan/remove/reduce calls in the generator passes allocate
 * their temporary storage from the stream's async memory pool on every
 * call -- steady pool traffic for buffers whose sizes are bounded by the
 * fixed distribution-buffer capacity. This arena is grown to the previous
 * call's recorded demand before any suballocation is handed out (nothing
 * from an earlier call is live by then, and both the release and the
 * growth are stream-ordered), so after the first call at a given demand
 * the temporaries come from here and steady-state calls make no pool
 * operations. Overflow within a call falls back to the stream pool --
 * correct, merely unoptimized -- and raises the recorded demand so the
 * next call grows past it.
 *
 * Host-memspace states never touch this; it stays empty there.
 */
struct GeneratorScratch
{
    DeviceVector<char> temp;
    std::size_t need{0};
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
