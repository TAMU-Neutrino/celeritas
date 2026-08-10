//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalEventTable.hh
//---------------------------------------------------------------------------//
#pragma once

#include <deque>
#include <optional>
#include <vector>

#include "corecel/OpaqueId.hh"
#include "corecel/Types.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Track event completion across process-wide optical transport lanes.
 *
 * Events are registered by their full run-global ordinal and routed
 * deterministically to a lane. Completion is latched only after the producer
 * closes the event, every submitted burst is absorbed, every submitted photon
 * is generated, a census after the final append clears the event, and the
 * lane's hits have been delivered through its ordinal.
 *
 * This class performs no synchronization. The owner must externally serialize
 * all calls when an instance is shared between threads.
 */
class OpticalEventTable
{
  public:
    //!@{
    //! \name Type aliases
    using LaneId = OpaqueId<struct OpticalEventLane_>;
    //!@}

  public:
    // Construct for a fixed number of lanes and admission bound
    OpticalEventTable(size_type num_lanes, size_type unresolved_limit);

    // Register an event before any submission and return its assigned lane
    LaneId register_event(long ordinal);

    // Append one burst to a registered, open event
    void submit_burst(long ordinal, size_type num_photons);

    // Close an event against any later submissions
    void close_event(long ordinal);

    // Consume the oldest submitted bursts on a lane
    void record_bursts_absorbed(LaneId lane, size_type count);

    // Update a lane's cumulative number of generated photons
    void record_generation_progress(LaneId lane, size_type total_generated);

    // Record a fresh census minimum, or no live event
    void record_census(LaneId lane, std::optional<long> min_live_ordinal);

    // Update the highest ordinal whose hit callback has completed on a lane
    void record_delivered_through(LaneId lane, long ordinal);

    // Whether an event has satisfied the full completion contract
    bool is_complete(long ordinal) const;

    //! Highest ordinal for which every earlier event is complete
    long completion_watermark() const { return completion_watermark_; }

    //! Number of registered events not yet complete
    size_type unresolved_count() const { return unresolved_count_; }

    //! Configured maximum number of unresolved events
    size_type unresolved_limit() const { return unresolved_limit_; }

    //! Whether another event can be admitted under the unresolved bound
    bool admission_available() const
    {
        return unresolved_count_ < unresolved_limit_;
    }

    //! Number of transport lanes
    size_type num_lanes() const { return lanes_.size(); }

  private:
    struct EventState
    {
        long ordinal{-1};
        LaneId lane;
        size_type submitted_bursts{0};
        size_type absorbed_bursts{0};
        size_type generation_target{0};
        size_type last_append_epoch{0};
        bool closed{false};
        bool complete{false};
    };

    struct LaneState
    {
        std::deque<long> pending_bursts;
        size_type submitted_photons{0};
        size_type generated_photons{0};
        size_type append_epoch{0};
        size_type census_epoch{0};
        bool census_valid{false};
        std::optional<long> min_live_ordinal;
        long delivered_through{-1};
    };

    EventState& event(long ordinal);
    LaneState& lane(LaneId lane);
    LaneState const& lane(LaneId lane) const;
    bool census_clears(EventState const& event) const;
    void update_event(EventState& event);
    void update_lane(LaneId lane);
    void advance_watermark();

    std::vector<LaneState> lanes_;
    // Front corresponds to completion_watermark_ + 1; gaps are unregistered
    std::deque<std::optional<EventState>> events_;
    size_type unresolved_limit_;
    size_type unresolved_count_{0};
    long completion_watermark_{-1};
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
