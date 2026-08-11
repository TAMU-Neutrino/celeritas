//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/FakeOpticalTransportLane.hh
//---------------------------------------------------------------------------//
#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "celeritas/optical/Types.hh"
#include "accel/detail/OpticalTransportLane.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
/*!
 * Deterministic gate for holding one fake lane operation in flight.
 */
class FakeOpticalLaneGate
{
  public:
    void wait()
    {
        std::unique_lock<std::mutex> lock{mutex_};
        entered_ = true;
        cv_.notify_all();
        cv_.wait(lock, [this] { return open_; });
    }

    bool wait_until_entered(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock{mutex_};
        return cv_.wait_for(lock, timeout, [this] { return entered_; });
    }

    void release()
    {
        std::lock_guard<std::mutex> lock{mutex_};
        open_ = true;
        cv_.notify_all();
    }

  private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool entered_{false};
    bool open_{false};
};

//---------------------------------------------------------------------------//
/*!
 * Shared observations from one fake lane.
 */
struct FakeOpticalLaneState
{
    struct Snapshot
    {
        size_type transported_bursts{0};
        size_type closed_events{0};
        std::vector<long> reseeded_events;
        size_type finalizations{0};
        std::thread::id owner;
    };

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock{mutex};
        return {transported_bursts,
                closed_events,
                reseeded_events,
                finalizations,
                owner};
    }

    mutable std::mutex mutex;
    size_type transported_bursts{0};
    size_type closed_events{0};
    std::vector<long> reseeded_events;
    size_type finalizations{0};
    std::thread::id owner;
};

//---------------------------------------------------------------------------//
/*!
 * Host-only lane implementation with deterministic delay and failure hooks.
 */
class FakeOpticalTransportLane final
    : public detail::OpticalTransportLaneInterface
{
  public:
    struct Config
    {
        std::chrono::microseconds delay{0};
        std::chrono::microseconds jitter{0};
        std::optional<long> failure_event;
        std::shared_ptr<FakeOpticalLaneGate> gate;
        std::optional<long> gated_event;
    };

    FakeOpticalTransportLane(Config config,
                             std::shared_ptr<FakeOpticalLaneState> state)
        : config_(std::move(config)), state_(std::move(state))
    {
    }

    detail::OpticalTransportLaneProgress
    transport(detail::OpticalTransportBurst const& burst) final
    {
        size_type const operation = this->record_call(false);
        if (config_.gate && config_.gated_event == burst.event)
        {
            config_.gate->wait();
        }
        this->delay(burst.event, operation);
        if (config_.failure_event == burst.event)
        {
            throw std::runtime_error{"injected fake optical lane failure"};
        }

        total_generated_ += burst.num_photons;
        detail::OpticalTransportLaneProgress result;
        result.total_generated = total_generated_;
        if (burst.num_photons > 0)
        {
            detail::OpticalTransportHitBatch batch;
            batch.event = burst.event;
            batch.hits.resize(burst.num_photons);
            auto const event_in_ring = static_cast<size_type>(
                burst.event % static_cast<long>(optical::event_ring));
            for (auto& hit : batch.hits)
            {
                hit.primary = id_cast<PrimaryId>(
                    event_in_ring << optical::event_shift);
            }
            result.hit_batches.push_back(std::move(batch));
        }
        result.census_fresh = true;
        return result;
    }

    detail::OpticalTransportLaneProgress close_event(long ordinal) final
    {
        size_type const operation = this->record_call(true);
        this->delay(ordinal, operation);
        if (config_.failure_event == ordinal)
        {
            throw std::runtime_error{"injected fake optical lane failure"};
        }

        detail::OpticalTransportLaneProgress result;
        result.total_generated = total_generated_;
        result.census_fresh = true;
        return result;
    }

    void reseed(long event_ordinal) final
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        state_->reseeded_events.push_back(event_ordinal);
    }

    void finalize() final
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        ++state_->finalizations;
    }

    MapStrDbl action_time() const final
    {
        return {{"fake-transport", static_cast<double>(total_generated_)}};
    }

  private:
    size_type record_call(bool close)
    {
        std::lock_guard<std::mutex> lock{state_->mutex};
        auto const this_thread = std::this_thread::get_id();
        if (state_->owner == std::thread::id{})
        {
            state_->owner = this_thread;
        }
        else if (state_->owner != this_thread)
        {
            throw std::runtime_error{"fake optical lane changed owner thread"};
        }

        if (close)
        {
            return ++state_->closed_events;
        }
        return ++state_->transported_bursts;
    }

    void delay(long ordinal, size_type operation) const
    {
        auto sleep_time = config_.delay;
        if (config_.jitter.count() > 0)
        {
            auto const key = static_cast<unsigned long long>(ordinal)
                                 * 0x9e3779b97f4a7c15ULL
                             + static_cast<unsigned long long>(operation);
            sleep_time += std::chrono::microseconds{
                static_cast<std::chrono::microseconds::rep>(
                    key
                    % (static_cast<unsigned long long>(config_.jitter.count())
                       + 1))};
        }
        if (sleep_time.count() > 0)
        {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    Config config_;
    std::shared_ptr<FakeOpticalLaneState> state_;
    size_type total_generated_{0};
};

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
