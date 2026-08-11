//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/OpticalEventTable.test.cc
//---------------------------------------------------------------------------//
#include "accel/detail/OpticalEventTable.hh"

#include "corecel/Assert.hh"
#include "celeritas/optical/Types.hh"

#include "celeritas_test.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
using OpticalEventTable = detail::OpticalEventTable;
using LaneId = OpticalEventTable::LaneId;

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, registration_precedes_submission)
{
    OpticalEventTable events{2, 8};

    EXPECT_THROW(events.submit_burst(0, 10), RuntimeError);
    EXPECT_EQ(LaneId{1}, events.register_event(1));
    EXPECT_THROW(events.register_event(1), RuntimeError);
    events.close_event(1);
    EXPECT_THROW(events.submit_burst(1, 10), RuntimeError);
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, requires_all_completion_conditions)
{
    OpticalEventTable events{1, 8};
    LaneId const lane = events.register_event(0);
    events.submit_burst(0, 10);
    events.close_event(0);

    EXPECT_FALSE(events.is_complete(0));
    events.record_bursts_absorbed(lane, 1);
    EXPECT_FALSE(events.is_complete(0));
    events.record_generation_progress(lane, 10);
    EXPECT_FALSE(events.is_complete(0));
    events.record_census(lane, std::nullopt);
    EXPECT_TRUE(events.is_transport_complete(0));
    EXPECT_FALSE(events.is_complete(0));
    events.record_delivered_through(lane, 0);
    EXPECT_TRUE(events.is_complete(0));
    EXPECT_EQ(0, events.completion_watermark());
    EXPECT_EQ(0, events.unresolved_count());
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, auto_flush_requires_close)
{
    OpticalEventTable events{1, 8};
    LaneId const lane = events.register_event(0);
    events.submit_burst(0, 10);
    events.record_bursts_absorbed(lane, 1);
    events.record_generation_progress(lane, 10);
    events.record_census(lane, std::nullopt);
    events.record_delivered_through(lane, 0);

    // All transport work so far is done, but the producer is still open
    EXPECT_FALSE(events.is_transport_complete(0));
    EXPECT_FALSE(events.is_complete(0));

    // A later auto-flush burst must invalidate the would-be completion
    events.submit_burst(0, 5);
    events.close_event(0);
    events.record_bursts_absorbed(lane, 1);
    events.record_generation_progress(lane, 15);
    EXPECT_FALSE(events.is_complete(0));

    // Only a census after the final append can complete the event
    events.record_census(lane, std::nullopt);
    EXPECT_TRUE(events.is_transport_complete(0));
    EXPECT_TRUE(events.is_complete(0));
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, out_of_order_interleaved_events)
{
    OpticalEventTable events{2, 8};
    LaneId const lane0 = events.register_event(0);
    LaneId const lane1 = events.register_event(1);
    events.register_event(2);
    events.register_event(3);

    for (long event = 0; event < 4; ++event)
    {
        events.submit_burst(event, static_cast<size_type>(event + 1));
        events.close_event(event);
    }

    // Finish lane 1 while lane 0 is still unresolved
    events.record_bursts_absorbed(lane1, 2);
    events.record_generation_progress(lane1, 6);
    events.record_census(lane1, std::nullopt);
    events.record_delivered_through(lane1, 3);

    EXPECT_FALSE(events.is_complete(0));
    EXPECT_TRUE(events.is_complete(1));
    EXPECT_FALSE(events.is_complete(2));
    EXPECT_TRUE(events.is_complete(3));
    EXPECT_EQ(-1, events.completion_watermark());
    EXPECT_EQ(2, events.unresolved_count());

    // The other lane can finish independently
    events.record_bursts_absorbed(lane0, 2);
    events.record_generation_progress(lane0, 4);
    events.record_census(lane0, std::nullopt);
    events.record_delivered_through(lane0, 2);
    EXPECT_EQ(3, events.completion_watermark());
    EXPECT_EQ(0, events.unresolved_count());
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, census_snapshot_invalidation)
{
    OpticalEventTable events{1, 8};
    LaneId const lane = events.register_event(0);
    events.submit_burst(0, 2);
    events.record_bursts_absorbed(lane, 1);
    events.record_generation_progress(lane, 2);
    events.record_census(lane, std::nullopt);
    events.record_delivered_through(lane, 0);

    // Any later append on the lane invalidates its census snapshot
    events.register_event(1);
    events.submit_burst(1, 3);
    events.close_event(0);
    EXPECT_FALSE(events.is_complete(0));

    events.record_census(lane, 1);
    EXPECT_TRUE(events.is_complete(0));
    EXPECT_FALSE(events.is_complete(1));
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, watermark_vs_per_event_completion)
{
    OpticalEventTable events{2, 8};
    LaneId const lane0 = events.register_event(0);
    LaneId const lane1 = events.register_event(1);
    events.close_event(0);
    events.close_event(1);

    events.record_census(lane1, std::nullopt);
    events.record_delivered_through(lane1, 1);
    EXPECT_TRUE(events.is_complete(1));
    EXPECT_EQ(-1, events.completion_watermark());

    events.record_census(lane0, std::nullopt);
    events.record_delivered_through(lane0, 0);
    EXPECT_TRUE(events.is_complete(0));
    EXPECT_EQ(1, events.completion_watermark());
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, admission_backpressure_bound)
{
    EXPECT_THROW((OpticalEventTable{1, 0}), RuntimeError);
    EXPECT_THROW((OpticalEventTable{1, optical::event_ring}), RuntimeError);

    OpticalEventTable events{2, 2};
    EXPECT_EQ(2, events.unresolved_limit());
    EXPECT_TRUE(events.admission_available());
    LaneId const lane0 = events.register_event(0);
    LaneId const lane1 = events.register_event(1);
    EXPECT_EQ(2, events.unresolved_count());
    EXPECT_FALSE(events.admission_available());
    EXPECT_THROW(events.register_event(2), RuntimeError);

    // Out-of-order completion releases admission capacity immediately
    events.close_event(1);
    events.record_census(lane1, std::nullopt);
    events.record_delivered_through(lane1, 1);
    EXPECT_TRUE(events.is_complete(1));
    EXPECT_EQ(1, events.unresolved_count());
    EXPECT_TRUE(events.admission_available());
    EXPECT_EQ(lane0, events.register_event(2));
}

//---------------------------------------------------------------------------//
TEST(OpticalEventTableTest, base_ordinal)
{
    OpticalEventTable events{2, 4, 100};
    EXPECT_EQ(99, events.completion_watermark());

    EXPECT_EQ(LaneId{1}, events.register_event(101));
    EXPECT_EQ(LaneId{0}, events.register_event(100));
    EXPECT_THROW(events.register_event(99), RuntimeError);

    for (long ordinal : {100, 101})
    {
        events.close_event(ordinal);
        events.record_census(LaneId{static_cast<size_type>(ordinal % 2)},
                             std::nullopt);
        events.record_delivered_through(
            LaneId{static_cast<size_type>(ordinal % 2)}, ordinal);
        EXPECT_TRUE(events.is_complete(ordinal));
    }
    EXPECT_EQ(101, events.completion_watermark());
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
