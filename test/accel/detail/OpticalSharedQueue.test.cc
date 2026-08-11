//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalSharedQueue.test.cc
//---------------------------------------------------------------------------//
#include "accel/detail/OpticalSharedQueue.hh"

#include "celeritas/optical/Types.hh"
#include "celeritas/setup/Control.hh"

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
TEST(OpticalSharedQueueTest, resolve_input_and_overrides)
{
    inp::OpticalStreaming streaming;
    EXPECT_FALSE(detail::resolve_optical_shared_queue(streaming, "", ""));
    EXPECT_EQ(8, setup::optical_streams(streaming, 8));

    streaming.shared_queue = true;
    streaming.lane_count = 3;
    streaming.unresolved_event_limit = 17;
    streaming.staged_bytes_limit = 4096;
    auto configured = detail::resolve_optical_shared_queue(streaming, "", "");
    EXPECT_EQ(3, configured.lanes);
    EXPECT_EQ(0, configured.base_ordinal);
    EXPECT_EQ(17, configured.unresolved_limit);
    EXPECT_EQ(4096, configured.staged_bytes_limit);
    EXPECT_EQ(3, setup::optical_streams(streaming, 8));

    auto overridden
        = detail::resolve_optical_shared_queue(streaming, "5", "1000");
    EXPECT_EQ(5, overridden.lanes);
    EXPECT_EQ(1000, overridden.base_ordinal);
    EXPECT_EQ(17, overridden.unresolved_limit);

    auto base_only
        = detail::resolve_optical_shared_queue(streaming, "", "2000");
    EXPECT_EQ(3, base_only.lanes);
    EXPECT_EQ(2000, base_only.base_ordinal);

    streaming.lane_count = 0;
    EXPECT_THROW(detail::resolve_optical_shared_queue(streaming, "", ""),
                 RuntimeError);
    streaming.lane_count = 1;
    streaming.unresolved_event_limit = optical::event_ring;
    EXPECT_THROW(detail::resolve_optical_shared_queue(streaming, "", ""),
                 RuntimeError);
    streaming.unresolved_event_limit = 1;
    streaming.staged_bytes_limit = 0;
    EXPECT_THROW(detail::resolve_optical_shared_queue(streaming, "", ""),
                 RuntimeError);
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
