//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalSharedQueue.cc
//---------------------------------------------------------------------------//
#include "OpticalSharedQueue.hh"

#include <charconv>
#include <limits>
#include <string>

#include "corecel/Assert.hh"
#include "corecel/sys/Environment.hh"

namespace celeritas
{
namespace detail
{
namespace
{
//---------------------------------------------------------------------------//
template<class T>
T parse_nonnegative(std::string_view value, char const* name)
{
    T result{};
    auto const parsed
        = std::from_chars(value.data(), value.data() + value.size(), result);
    CELER_VALIDATE(
        parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size(),
        << "invalid " << name << " value '" << value << "'");
    return result;
}

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
/*!
 * Parse explicit shared-queue settings.
 */
OpticalSharedQueueConfig
parse_optical_shared_queue(std::string_view lanes, std::string_view base)
{
    if (lanes.empty())
    {
        CELER_VALIDATE(base.empty(),
                       << "CELER_OPTICAL_SHARED_BASE_ORDINAL requires "
                          "CELER_OPTICAL_SHARED_QUEUE");
        return {};
    }

    auto const parsed_lanes
        = parse_nonnegative<unsigned long long>(lanes, "shared lane count");
    CELER_VALIDATE(parsed_lanes > 0
                       && parsed_lanes <= std::numeric_limits<size_type>::max(),
                   << "shared optical lane count must be positive and fit "
                      "in size_type");

    unsigned long long parsed_base{0};
    if (!base.empty())
    {
        parsed_base = parse_nonnegative<unsigned long long>(
            base, "shared base ordinal");
    }
    CELER_VALIDATE(parsed_base <= static_cast<unsigned long long>(
                       std::numeric_limits<long>::max()),
                   << "shared optical base ordinal is too large");

    return {static_cast<size_type>(parsed_lanes),
            static_cast<long>(parsed_base)};
}

//---------------------------------------------------------------------------//
/*!
 * Read provisional shared-queue settings from the environment.
 */
OpticalSharedQueueConfig optical_shared_queue_config()
{
    return parse_optical_shared_queue(
        celeritas::getenv("CELER_OPTICAL_SHARED_QUEUE"),
        celeritas::getenv("CELER_OPTICAL_SHARED_BASE_ORDINAL"));
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
