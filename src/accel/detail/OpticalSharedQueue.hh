//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalSharedQueue.hh
//! \sa OpticalSharedQueue.test.cc
//---------------------------------------------------------------------------//
#pragma once

#include <string_view>

#include "corecel/Types.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Provisional environment configuration for process-wide optical transport.
 */
struct OpticalSharedQueueConfig
{
    size_type lanes{0};
    long base_ordinal{0};

    explicit operator bool() const { return lanes > 0; }
};

// Parse explicit values for host testing
OpticalSharedQueueConfig
parse_optical_shared_queue(std::string_view lanes, std::string_view base);

// Read the provisional shared-queue environment variables
OpticalSharedQueueConfig optical_shared_queue_config();

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
