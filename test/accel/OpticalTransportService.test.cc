//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/OpticalTransportService.test.cc
//---------------------------------------------------------------------------//
#include "accel/detail/OpticalTransportService.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

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
class HitCollector
{
  public:
    Service::HitCallback callback()
    {
        return [this](long ordinal,
                      std::vector<detail::OpticalTransportHit> const& hits) {
            std::lock_guard<std::mutex> lock{mutex_};
            ++callbacks_[ordinal];
            for (auto const& hit : hits)
            {
                if (hit.event != ordinal)
                {
                    throw std::runtime_error{"hit delivered to wrong event"};
                }
                photons_[ordinal] += hit.num_photons;
                total_photons_ += hit.num_photons;
            }
        };
    }

    size_type photons(long ordinal) const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        auto iter = photons_.find(ordinal);
        return iter == photons_.end() ? 0 : iter->second;
    }

    size_type callbacks(long ordinal) const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        auto iter = callbacks_.find(ordinal);
        return iter == callbacks_.end() ? 0 : iter->second;
    }

    size_type total_photons() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return total_photons_;
    }

  private:
    mutable std::mutex mutex_;
    std::unordered_map<long, size_type> photons_;
    std::unordered_map<long, size_type> callbacks_;
    size_type total_photons_{0};
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
TEST(OpticalTransportServiceTest, concurrent_out_of_order_producers)
{
    constexpr size_type num_events = 64;
    constexpr size_type num_lanes = 4;
    FakeLaneSetup fake{num_lanes};
    for (auto& config : fake.configs)
    {
        config.delay = 10us;
        config.jitter = 40us;
    }
    HitCollector hits;
    Service service{{num_lanes, 128, 64}, fake.factory(), hits.callback()};

    std::vector<long> order(num_events);
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 rng{12345};
    std::shuffle(order.begin(), order.end(), rng);

    {
        std::vector<std::future<void>> producers;
        producers.reserve(num_events);
        for (long ordinal : order)
        {
            auto token = service.make_producer();
            producers.push_back(std::async(
                std::launch::async,
                [token = std::move(token), ordinal]() mutable {
                    std::this_thread::sleep_for(
                        std::chrono::microseconds{(ordinal * 17) % 101});
                    LaneId const lane = token.register_event(ordinal);
                    if (*lane
                        != static_cast<size_type>(
                            ordinal % static_cast<long>(num_lanes)))
                    {
                        throw std::runtime_error{
                            "nondeterministic lane route"};
                    }
                    token.submit_burst(ordinal, 1, 1);
                    if (ordinal % 3 == 0)
                    {
                        std::this_thread::yield();
                    }
                    token.close_event(ordinal);
                    token.wait_until_complete(ordinal);
                }));
        }
        for (auto& producer : producers)
        {
            EXPECT_NO_THROW(producer.get());
        }
    }

    service.drain_and_stop();
    auto const stats = service.statistics();
    EXPECT_EQ(0, stats.unresolved_events);
    EXPECT_EQ(63, stats.completion_watermark);
    EXPECT_EQ(num_events, hits.total_photons());
    for (long ordinal = 0; ordinal < static_cast<long>(num_events); ++ordinal)
    {
        EXPECT_EQ(1, hits.photons(ordinal)) << ordinal;
    }
    for (auto const& state : fake.states)
    {
        EXPECT_NE(std::thread::id{}, state->snapshot().owner);
    }
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, empty_events)
{
    FakeLaneSetup fake{2};
    HitCollector hits;
    Service service{{2, 16, 8}, fake.factory(), hits.callback()};
    {
        auto token = service.make_producer();
        for (long ordinal = 0; ordinal < 12; ++ordinal)
        {
            token.register_event(ordinal);
            token.close_event(ordinal);
        }
        for (long ordinal = 0; ordinal < 12; ++ordinal)
        {
            token.wait_until_complete(ordinal);
            EXPECT_GE(hits.callbacks(ordinal), 1);
            EXPECT_EQ(0, hits.photons(ordinal));
        }
    }
    service.drain_and_stop();
    EXPECT_EQ(11, service.statistics().completion_watermark);
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, multi_burst_mid_event_pause)
{
    FakeLaneSetup fake{1};
    fake.configs[0].delay = 50us;
    HitCollector hits;
    Service service{{1, 4, 8}, fake.factory(), hits.callback()};
    {
        auto token = service.make_producer();
        token.register_event(0);
        token.submit_burst(0, 2, 2);

        Service::PumpResult first;
        EXPECT_TRUE(wait_until([&] {
            first = service.pump(0);
            return first.pumped;
        }));
        EXPECT_FALSE(first.complete);
        EXPECT_FALSE(token.is_complete(0));
        EXPECT_EQ(2, hits.photons(0));

        // The producer remains open across an auto-flush-shaped pause
        std::this_thread::yield();
        token.submit_burst(0, 3, 3);
        token.close_event(0);
        token.wait_until_complete(0);
        EXPECT_EQ(5, hits.photons(0));
    }
    service.drain_and_stop();
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, event_backpressure_engages_and_releases)
{
    FakeLaneSetup fake{1};
    Service service{{1, 1, 4}, fake.factory()};
    {
        auto first = service.make_producer();
        first.register_event(0);
        first.close_event(0);

        auto second = service.make_producer();
        auto blocked = std::async(std::launch::async,
                                  [token = std::move(second)]() mutable {
                                      token.register_event(1);
                                      token.close_event(1);
                                      token.wait_until_complete(1);
                                  });

        EXPECT_TRUE(wait_until(
            [&] { return service.statistics().event_admission_waits == 1; }));
        EXPECT_EQ(std::future_status::timeout, blocked.wait_for(0ms));

        first.wait_until_complete(0);
        EXPECT_NO_THROW(blocked.get());
    }
    service.drain_and_stop();
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, admission_window_keeps_gap_open)
{
    FakeLaneSetup fake{1};
    Service service{{1, 2, 4}, fake.factory()};
    {
        auto high = service.make_producer();
        high.register_event(1);
        high.close_event(1);

        auto far = service.make_producer();
        auto blocked = std::async(std::launch::async,
                                  [token = std::move(far)]() mutable {
                                      token.register_event(2);
                                      token.close_event(2);
                                      token.wait_until_complete(2);
                                  });
        EXPECT_TRUE(wait_until(
            [&] { return service.statistics().event_admission_waits == 1; }));

        // The missing low ordinal remains admissible despite event 1
        auto low = service.make_producer();
        low.register_event(0);
        low.close_event(0);
        low.wait_until_complete(0);
        high.wait_until_complete(1);
        EXPECT_NO_THROW(blocked.get());
    }
    service.drain_and_stop();
    EXPECT_EQ(2, service.statistics().completion_watermark);
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest,
     staged_bytes_backpressure_engages_and_releases)
{
    auto gate = std::make_shared<FakeOpticalLaneGate>();
    FakeLaneSetup fake{1};
    fake.configs[0].gate = gate;
    fake.configs[0].gated_event = 0;
    Service service{{1, 8, 1}, fake.factory()};
    {
        auto first = service.make_producer();
        auto second = service.make_producer();
        auto third = service.make_producer();
        first.register_event(0);
        second.register_event(1);
        third.register_event(2);
        EXPECT_THROW(first.submit_burst(0, 1, 0), RuntimeError);
        EXPECT_THROW(first.submit_burst(0, 1, 2), RuntimeError);
        first.submit_burst(0, 1, 1);
        if (!gate->wait_until_entered(2s))
        {
            gate->release();
            FAIL() << "fake lane did not enter the deterministic gate";
        }
        first.close_event(0);
        second.submit_burst(1, 1, 1);
        second.close_event(1);

        auto blocked = std::async(std::launch::async,
                                  [token = std::move(third)]() mutable {
                                      token.submit_burst(2, 1, 1);
                                      token.close_event(2);
                                      token.wait_until_complete(2);
                                  });
        bool const engaged = wait_until(
            [&] { return service.statistics().staged_bytes_waits == 1; });
        gate->release();
        EXPECT_TRUE(engaged);
        EXPECT_NO_THROW(first.wait_until_complete(0));
        EXPECT_NO_THROW(second.wait_until_complete(1));
        EXPECT_NO_THROW(blocked.get());
        EXPECT_EQ(1, service.statistics().max_staged_bytes);
    }
    service.drain_and_stop();
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, lane_failure_fans_out)
{
    auto gate = std::make_shared<FakeOpticalLaneGate>();
    FakeLaneSetup fake{1};
    fake.configs[0].gate = gate;
    fake.configs[0].gated_event = 0;
    fake.configs[0].failure_event = 0;
    Service service{{1, 1, 1}, fake.factory()};
    {
        auto first = service.make_producer();
        auto second = service.make_producer();
        auto third = service.make_producer();
        first.register_event(0);
        first.submit_burst(0, 1, 1);
        first.close_event(0);
        if (!gate->wait_until_entered(2s))
        {
            gate->release();
            FAIL() << "fake lane did not enter the deterministic gate";
        }

        auto blocked = std::async(std::launch::async,
                                  [token = std::move(second)]() mutable {
                                      token.register_event(1);
                                  });
        bool const waiting = wait_until(
            [&] { return service.statistics().event_admission_waits == 1; });
        gate->release();
        EXPECT_TRUE(waiting);
        EXPECT_THROW(blocked.get(), std::runtime_error);
        EXPECT_THROW(first.is_complete(0), std::runtime_error);
        EXPECT_THROW(third.register_event(2), std::runtime_error);
    }
    EXPECT_THROW(service.drain_and_stop(), std::runtime_error);
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, random_producer_destruction_order)
{
    constexpr size_type num_events = 32;
    FakeLaneSetup fake{4};
    for (auto& config : fake.configs)
    {
        config.delay = 10us;
        config.jitter = 30us;
    }
    HitCollector hits;
    Service service{{4, 64, 32}, fake.factory(), hits.callback()};

    std::vector<std::unique_ptr<Service::ProducerToken>> producers;
    producers.reserve(num_events);
    for (long ordinal = 0; ordinal < static_cast<long>(num_events); ++ordinal)
    {
        auto token = std::make_unique<Service::ProducerToken>(
            service.make_producer());
        token->register_event(ordinal);
        token->submit_burst(ordinal, 1, 1);
        token->close_event(ordinal);
        producers.push_back(std::move(token));
    }

    std::mt19937 rng{9876};
    std::shuffle(producers.begin(), producers.end(), rng);
    for (auto& producer : producers)
    {
        producer.reset();
    }

    service.drain_and_stop();
    EXPECT_EQ(num_events, hits.total_photons());
    EXPECT_EQ(0, service.statistics().active_producers);
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, producer_abandonment_is_terminal)
{
    FakeLaneSetup fake{1};
    Service service{{1, 2, 2}, fake.factory()};
    {
        auto token = service.make_producer();
        token.register_event(0);
    }

    EXPECT_THROW(service.make_producer(), std::runtime_error);
    EXPECT_THROW(service.drain_and_stop(), std::runtime_error);
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, drain_with_in_flight_work)
{
    constexpr size_type num_events = 24;
    FakeLaneSetup fake{2};
    for (auto& config : fake.configs)
    {
        config.delay = 100us;
        config.jitter = 100us;
    }
    HitCollector hits;
    Service service{{2, 32, 24}, fake.factory(), hits.callback()};
    {
        auto token = service.make_producer();
        for (long ordinal = 0; ordinal < static_cast<long>(num_events);
             ++ordinal)
        {
            token.register_event(ordinal);
            token.submit_burst(ordinal, 1, 1);
            token.close_event(ordinal);
        }
        EXPECT_THROW(service.drain_and_stop(), RuntimeError);
    }

    service.drain_and_stop();
    auto const stats = service.statistics();
    EXPECT_TRUE(stats.stopped);
    EXPECT_EQ(0, stats.unresolved_events);
    EXPECT_EQ(static_cast<long>(num_events - 1), stats.completion_watermark);
    EXPECT_EQ(num_events, hits.total_photons());
}

//---------------------------------------------------------------------------//
TEST(OpticalTransportServiceTest, bounded_memory_soak)
{
    constexpr long num_events = 100000;
    constexpr long batch_size = 512;
    FakeLaneSetup fake{2};
    std::atomic<size_type> hit_photons{0};
    Service service{
        {2, batch_size, batch_size},
        fake.factory(),
        [&hit_photons](long,
                       std::vector<detail::OpticalTransportHit> const& hits) {
            for (auto const& hit : hits)
            {
                hit_photons.fetch_add(hit.num_photons,
                                      std::memory_order_relaxed);
            }
        }};
    {
        auto token = service.make_producer();
        for (long begin = 0; begin < num_events; begin += batch_size)
        {
            long const end = std::min(begin + batch_size, num_events);
            for (long ordinal = begin; ordinal < end; ++ordinal)
            {
                token.register_event(ordinal);
                token.submit_burst(ordinal, 1, 1);
                token.close_event(ordinal);
            }
            for (long ordinal = begin; ordinal < end; ++ordinal)
            {
                token.wait_until_complete(ordinal);
            }
        }
    }
    service.drain_and_stop();

    auto const stats = service.statistics();
    EXPECT_EQ(num_events, static_cast<long>(hit_photons.load()));
    EXPECT_EQ(num_events - 1, stats.completion_watermark);
    EXPECT_EQ(0, stats.unresolved_events);
    EXPECT_EQ(0, stats.resident_events);
    EXPECT_LE(stats.max_unresolved_events, static_cast<size_type>(batch_size));
    EXPECT_LE(stats.max_resident_events, static_cast<size_type>(batch_size));
    EXPECT_LE(stats.max_staged_bytes, static_cast<size_type>(batch_size));
    EXPECT_LE(stats.max_mailbox_hits, static_cast<size_type>(batch_size));
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
