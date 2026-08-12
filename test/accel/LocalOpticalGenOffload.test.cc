//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/LocalOpticalGenOffload.test.cc
//---------------------------------------------------------------------------//
#include "accel/LocalOpticalGenOffload.hh"

#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "accel/detail/OpticalTransportService.hh"

#include "celeritas_test.hh"

#include "detail/FakeOpticalTransportLane.hh"

namespace celeritas
{
namespace test
{
namespace
{
using Service = detail::OpticalTransportService;
using LaneId = Service::LaneId;
using namespace std::chrono_literals;

//---------------------------------------------------------------------------//
class FakeLaneSetup
{
  public:
    explicit FakeLaneSetup(size_type count) : configs(count), states(count)
    {
        for (auto& state : states)
        {
            state = std::make_shared<FakeOpticalLaneState>();
        }
    }

    Service::LaneFactory factory() const
    {
        auto configs_copy = configs;
        auto states_copy = states;
        return [configs = std::move(configs_copy),
                states = std::move(states_copy)](LaneId lane) {
            return std::make_unique<FakeOpticalTransportLane>(configs[*lane],
                                                              states[*lane]);
        };
    }

    std::vector<FakeOpticalTransportLane::Config> configs;
    std::vector<std::shared_ptr<FakeOpticalLaneState>> states;
};

//---------------------------------------------------------------------------//
template<class F>
bool wait_until(F&& predicate, std::chrono::milliseconds timeout = 2s)
{
    auto const deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (predicate())
        {
            return true;
        }
        std::this_thread::sleep_for(100us);
    }
    return predicate();
}

//---------------------------------------------------------------------------//
}  // namespace

//---------------------------------------------------------------------------//
class LocalOpticalGenOffloadTestAccess
{
  public:
    static LocalOpticalGenOffload make_facade(std::shared_ptr<Service> service)
    {
        return LocalOpticalGenOffload::MakeSharedQueueTestFacade(
            std::move(service));
    }
};

//---------------------------------------------------------------------------//
TEST(LocalOpticalGenOffloadTest, reports_out_of_order_completion)
{
    auto gate = std::make_shared<FakeOpticalLaneGate>();
    FakeLaneSetup fake{2};
    fake.configs[0].gate = gate;
    fake.configs[0].gated_close_event = 0;
    auto service
        = std::make_shared<Service>(Service::Options{2, 4, 4}, fake.factory());
    auto facade = LocalOpticalGenOffloadTestAccess::make_facade(service);
    auto control = facade.GetSharedTransportService();

    facade.InitializeEvent(0);
    facade.StageStreaming();
    bool const lane_blocked = gate->wait_until_entered(2s);

    facade.InitializeEvent(1);
    facade.StageStreaming();
    bool const later_completed = lane_blocked && wait_until([&] {
                                     control.TryPump();
                                     return facade.IsSharedEventComplete(1);
                                 });
    bool const earlier_completed = facade.IsSharedEventComplete(0);
    auto first_report = later_completed ? facade.TakeCompletedSharedEvents()
                                        : std::vector<long>{};
    long const prefix_before = facade.PumpStreaming();

    // Never leave the lane blocked if an expectation above fails.
    gate->release();
    EXPECT_TRUE(lane_blocked);
    EXPECT_TRUE(later_completed);
    EXPECT_FALSE(earlier_completed);
    EXPECT_VEC_EQ((std::vector<long>{1}), first_report);
    EXPECT_EQ(-1, prefix_before);
    EXPECT_TRUE(facade.TakeCompletedSharedEvents().empty());

    facade.WaitForSharedEventsThrough(1);
    EXPECT_VEC_EQ((std::vector<long>{0}), facade.TakeCompletedSharedEvents());
    EXPECT_EQ(1, facade.PumpStreaming());
    EXPECT_TRUE(facade.IsSharedEventComplete(0));
    EXPECT_TRUE(control.IsComplete(1));

    facade.Finalize();
    control.Drain();
}

//---------------------------------------------------------------------------//
TEST(LocalOpticalGenOffloadTest, rejects_reporting_after_prefix_consumption)
{
    FakeLaneSetup fake{1};
    auto service
        = std::make_shared<Service>(Service::Options{1, 2, 2}, fake.factory());
    auto facade = LocalOpticalGenOffloadTestAccess::make_facade(service);

    facade.InitializeEvent(0);
    facade.StageStreaming();
    ASSERT_TRUE(wait_until([&] { return facade.PumpStreaming() == 0; }));

    EXPECT_THROW(facade.TakeCompletedSharedEvents(), RuntimeError);
    facade.Finalize();
}

//---------------------------------------------------------------------------//
TEST(LocalOpticalGenOffloadTest, wait_releases_facade_mutex)
{
    auto gate = std::make_shared<FakeOpticalLaneGate>();
    FakeLaneSetup fake{1};
    fake.configs[0].gate = gate;
    fake.configs[0].gated_close_event = 0;
    auto service
        = std::make_shared<Service>(Service::Options{1, 2, 2}, fake.factory());
    std::promise<LocalOpticalGenOffload*> facade_ready;
    std::promise<bool> lane_entered;
    std::promise<void> wait_started;
    std::promise<void> allow_finalize;
    auto facade_future = facade_ready.get_future();
    auto lane_future = lane_entered.get_future();
    auto started = wait_started.get_future();
    auto finalize = allow_finalize.get_future();
    auto owner = std::async(std::launch::async, [&] {
        auto facade = LocalOpticalGenOffloadTestAccess::make_facade(service);
        facade.InitializeEvent(0);
        facade.StageStreaming();
        lane_entered.set_value(gate->wait_until_entered(2s));
        facade_ready.set_value(&facade);
        wait_started.set_value();
        facade.WaitForSharedEventsThrough(0);
        finalize.wait();
        facade.Finalize();
    });
    auto* facade = facade_future.get();
    bool const lane_blocked = lane_future.get();
    started.wait();
    std::this_thread::sleep_for(50ms);

    auto module_pump = std::async(std::launch::async,
                                  [&] { return facade->PumpStreaming(); });
    bool const facade_mutex_released = module_pump.wait_for(100ms)
                                       == std::future_status::ready;

    // Always release the lane so failures cannot strand either future.
    gate->release();
    EXPECT_TRUE(lane_blocked);
    EXPECT_TRUE(facade_mutex_released);
    module_pump.get();
    allow_finalize.set_value();
    owner.get();
}

//---------------------------------------------------------------------------//
TEST(LocalOpticalGenOffloadTest, explicit_process_drain_lifecycle)
{
    FakeLaneSetup fake{1};
    auto service
        = std::make_shared<Service>(Service::Options{1, 2, 2}, fake.factory());
    auto facade = LocalOpticalGenOffloadTestAccess::make_facade(service);

    {
        auto control = facade.GetSharedTransportService();
        // A process drain with an active producer is rejected, leaving the
        // service accepting work so the caller can retry after worker stop.
        EXPECT_THROW(control.Drain(), RuntimeError);

        facade.InitializeEvent(0);
        facade.StageStreaming();
        facade.WaitForSharedEventsThrough(0);
        facade.Finalize();
        EXPECT_FALSE(service->statistics().stopped);
        EXPECT_EQ(0, fake.states[0]->snapshot().finalizations);

        control.Drain();
        EXPECT_TRUE(service->statistics().stopped);
        EXPECT_EQ(1, fake.states[0]->snapshot().finalizations);
        control.Drain();
        EXPECT_EQ(1, fake.states[0]->snapshot().finalizations);
    }

    // Releasing the retained control after explicit drain calls the legacy
    // registry fallback, which sees the stopped service and is a no-op.
    EXPECT_EQ(1, fake.states[0]->snapshot().finalizations);
}

//---------------------------------------------------------------------------//
TEST(LocalOpticalGenOffloadTest, auto_drain_without_process_handle)
{
    FakeLaneSetup fake{1};
    auto service
        = std::make_shared<Service>(Service::Options{1, 2, 2}, fake.factory());
    auto facade = LocalOpticalGenOffloadTestAccess::make_facade(service);

    facade.InitializeEvent(0);
    facade.StageStreaming();
    facade.Finalize();

    EXPECT_TRUE(service->statistics().stopped);
    EXPECT_EQ(1, fake.states[0]->snapshot().finalizations);
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
