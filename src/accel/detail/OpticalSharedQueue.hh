//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalSharedQueue.hh
//! \sa OpticalSharedQueue.test.cc
//---------------------------------------------------------------------------//
#pragma once

#include <optional>
#include <string_view>

#include "corecel/Types.hh"
#include "celeritas/inp/Control.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Resolved configuration for process-wide optical transport.
 */
struct OpticalSharedQueueConfig
{
    size_type lanes{0};
    long base_ordinal{0};
    size_type unresolved_limit{0};
    std::optional<size_type> staged_bytes_limit;

    explicit operator bool() const { return lanes > 0; }
};

// Parse explicit values for host testing
OpticalSharedQueueConfig
parse_optical_shared_queue(std::string_view lanes, std::string_view base);

// Resolve input configuration with explicit environment override values
OpticalSharedQueueConfig resolve_optical_shared_queue(
    inp::OpticalStreaming const&, std::string_view lanes, std::string_view base);

// Resolve input configuration with process environment overrides
OpticalSharedQueueConfig
optical_shared_queue_config(inp::OpticalStreaming const&);

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
