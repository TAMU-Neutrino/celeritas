//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalSharedQueue.test.cc
//---------------------------------------------------------------------------//
#include "accel/detail/OpticalSharedQueue.hh"

#include "celeritas_test.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
TEST(OpticalSharedQueueTest, parse)
{
    EXPECT_FALSE(detail::parse_optical_shared_queue("", ""));

    auto const configured = detail::parse_optical_shared_queue("4", "1000");
    EXPECT_EQ(4, configured.lanes);
    EXPECT_EQ(1000, configured.base_ordinal);

    auto const default_base = detail::parse_optical_shared_queue("2", "");
    EXPECT_EQ(2, default_base.lanes);
    EXPECT_EQ(0, default_base.base_ordinal);

    EXPECT_THROW(detail::parse_optical_shared_queue("0", ""), RuntimeError);
    EXPECT_THROW(detail::parse_optical_shared_queue("-1", ""), RuntimeError);
    EXPECT_THROW(detail::parse_optical_shared_queue("2x", ""), RuntimeError);
    EXPECT_THROW(detail::parse_optical_shared_queue("", "1"), RuntimeError);
    EXPECT_THROW(detail::parse_optical_shared_queue("2", "-1"), RuntimeError);
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
