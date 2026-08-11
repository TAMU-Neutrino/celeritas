//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/TimeOutput.test.cc
//---------------------------------------------------------------------------//
#include "accel/TimeOutput.hh"

#include "corecel/io/JsonPimpl.hh"

#include "celeritas_test.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
TEST(TimeOutputTest, lane_actions_and_worker_events)
{
    TimeOutput output{3, 2};
    output.RecordActionTime(0, {{"generate", 1.25}});
    output.RecordActionTime(1, {{"generate", 2.5}});
    output.RecordActionTime({});
    output.RecordEventTime(0.5);
    output.RecordSetupTime(0.25);
    output.RecordTotalTime(1.0);

    JsonPimpl json;
    output.output(&json);

    ASSERT_EQ(2, json.obj["actions"].size());
    EXPECT_EQ(1.25, json.obj["actions"][0]["generate"]);
    EXPECT_EQ(2.5, json.obj["actions"][1]["generate"]);
    ASSERT_EQ(3, json.obj["events"].size());
    EXPECT_EQ(0.5, json.obj["events"][0][0]);
    EXPECT_EQ("lane", json.obj["_index"]["actions"]);
    EXPECT_EQ("thread", json.obj["_index"]["events"]);
    EXPECT_EQ(0.25, json.obj["setup"]);
    EXPECT_EQ(1.0, json.obj["total"]);

    EXPECT_THROW(output.RecordActionTime(2, {}), RuntimeError);
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
