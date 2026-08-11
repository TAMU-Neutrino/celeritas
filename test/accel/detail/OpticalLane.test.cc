//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalLane.test.cc
//---------------------------------------------------------------------------//
#include "accel/detail/OpticalLane.hh"

#include <type_traits>

#include "celeritas_test.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
TEST(OpticalLaneTest, transport_interface)
{
    EXPECT_TRUE((std::is_base_of_v<detail::OpticalTransportLaneInterface,
                                   detail::OpticalLane>));
    EXPECT_TRUE(std::has_virtual_destructor_v<detail::OpticalLane>);
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
