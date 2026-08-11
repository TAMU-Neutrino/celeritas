//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalEventTable.cc
//---------------------------------------------------------------------------//
#include "OpticalEventTable.hh"

#include "corecel/Assert.hh"
#include "celeritas/optical/Types.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Construct for a fixed number of lanes and admission bound.
 */
OpticalEventTable::OpticalEventTable(
    size_type num_lanes, size_type unresolved_limit, long base_ordinal)
    : lanes_(num_lanes)
    , unresolved_limit_(unresolved_limit)
    , completion_watermark_(base_ordinal > 0 ? base_ordinal - 1 : -1)
{
    CELER_VALIDATE(base_ordinal >= 0,
                   << "invalid negative optical base ordinal " << base_ordinal);
    CELER_VALIDATE(num_lanes > 0,
                   << "optical event table requires at least one lane");
    CELER_VALIDATE(unresolved_limit > 0,
                   << "optical event table requires a positive unresolved "
                      "event limit");
    CELER_VALIDATE(unresolved_limit < optical::event_ring,
                   << "optical event table unresolved limit "
                   << unresolved_limit << " must be less than event ring "
                   << optical::event_ring);
}

//---------------------------------------------------------------------------//
/*!
 * Register an event before any submission and return its assigned lane.
 */
auto OpticalEventTable::register_event(long ordinal) -> LaneId
{
    CELER_VALIDATE(ordinal >= 0,
                   << "invalid negative optical event ordinal " << ordinal);
    CELER_VALIDATE(ordinal > completion_watermark_,
                   << "optical event " << ordinal
                   << " has already completed or been retired");
    CELER_VALIDATE(this->admission_available(),
                   << "optical event table admission bound of "
                   << unresolved_limit_ << " unresolved events was reached");

    long const first_ordinal = completion_watermark_ + 1;
    size_type const offset = static_cast<size_type>(ordinal - first_ordinal);
    CELER_VALIDATE(offset < optical::event_ring,
                   << "optical event " << ordinal << " is outside the "
                   << optical::event_ring << "-ordinal live event window");

    if (offset >= events_.size())
    {
        events_.resize(offset + 1);
    }
    CELER_VALIDATE(!events_[offset],
                   << "optical event " << ordinal << " is already registered");

    LaneId const lane_id{
        static_cast<size_type>(ordinal % static_cast<long>(lanes_.size()))};
    auto const& lane_state = this->lane(lane_id);
    EventState state;
    state.ordinal = ordinal;
    state.lane = lane_id;
    state.generation_target = lane_state.submitted_photons;
    state.last_append_epoch = lane_state.append_epoch;
    events_[offset] = state;
    ++unresolved_count_;
    return lane_id;
}

//---------------------------------------------------------------------------//
/*!
 * Append one burst to a registered, open event.
 */
void OpticalEventTable::submit_burst(long ordinal, size_type num_photons)
{
    auto& event_state = this->event(ordinal);
    CELER_VALIDATE(
        !event_state.closed,
        << "cannot submit a burst to closed optical event " << ordinal);

    auto& lane_state = this->lane(event_state.lane);
    ++lane_state.append_epoch;
    lane_state.census_valid = false;
    lane_state.min_live_ordinal.reset();
    lane_state.pending_bursts.push_back(ordinal);
    lane_state.submitted_photons += num_photons;

    ++event_state.submitted_bursts;
    event_state.generation_target = lane_state.submitted_photons;
    event_state.last_append_epoch = lane_state.append_epoch;
}

//---------------------------------------------------------------------------//
/*!
 * Close an event against any later submissions.
 */
void OpticalEventTable::close_event(long ordinal)
{
    auto& event_state = this->event(ordinal);
    CELER_VALIDATE(!event_state.closed,
                   << "optical event " << ordinal << " is already closed");
    event_state.closed = true;
    this->update_event(event_state);
    this->advance_watermark();
}

//---------------------------------------------------------------------------//
/*!
 * Consume the oldest submitted bursts on a lane.
 */
void OpticalEventTable::record_bursts_absorbed(LaneId lane_id, size_type count)
{
    auto& lane_state = this->lane(lane_id);
    CELER_VALIDATE(count <= lane_state.pending_bursts.size(),
                   << "cannot absorb " << count << " optical bursts on lane "
                   << lane_id << ": only " << lane_state.pending_bursts.size()
                   << " are pending");

    for (size_type i = 0; i < count; ++i)
    {
        long const ordinal = lane_state.pending_bursts.front();
        lane_state.pending_bursts.pop_front();
        auto& event_state = this->event(ordinal);
        ++event_state.absorbed_bursts;
        this->update_event(event_state);
    }
    this->advance_watermark();
}

//---------------------------------------------------------------------------//
/*!
 * Update a lane's cumulative number of generated photons.
 */
void OpticalEventTable::record_generation_progress(LaneId lane_id,
                                                   size_type total_generated)
{
    auto& lane_state = this->lane(lane_id);
    CELER_VALIDATE(total_generated >= lane_state.generated_photons,
                   << "optical generation progress cannot move backward on "
                      "lane "
                   << lane_id);
    CELER_VALIDATE(total_generated <= lane_state.submitted_photons,
                   << "optical generation progress " << total_generated
                   << " exceeds " << lane_state.submitted_photons
                   << " submitted photons on lane " << lane_id);
    lane_state.generated_photons = total_generated;
    this->update_lane(lane_id);
}

//---------------------------------------------------------------------------//
/*!
 * Record a fresh census minimum, or no live event.
 */
void OpticalEventTable::record_census(LaneId lane_id,
                                      std::optional<long> min_live_ordinal)
{
    CELER_VALIDATE(!min_live_ordinal || *min_live_ordinal >= 0,
                   << "invalid negative optical census ordinal");
    auto& lane_state = this->lane(lane_id);
    lane_state.census_epoch = lane_state.append_epoch;
    lane_state.census_valid = true;
    lane_state.min_live_ordinal = min_live_ordinal;
    this->update_lane(lane_id);
}

//---------------------------------------------------------------------------//
/*!
 * Update the highest ordinal whose hit callback has completed on a lane.
 */
void OpticalEventTable::record_delivered_through(LaneId lane_id, long ordinal)
{
    CELER_VALIDATE(ordinal >= -1,
                   << "invalid optical delivered-through ordinal " << ordinal);
    auto& lane_state = this->lane(lane_id);
    CELER_VALIDATE(ordinal >= lane_state.delivered_through,
                   << "optical delivered-through cursor cannot move backward "
                      "on lane "
                   << lane_id);
    lane_state.delivered_through = ordinal;
    this->update_lane(lane_id);
}

//---------------------------------------------------------------------------//
/*!
 * Whether an event has satisfied the full completion contract.
 */
bool OpticalEventTable::is_complete(long ordinal) const
{
    if (ordinal < 0)
    {
        return false;
    }
    if (ordinal <= completion_watermark_)
    {
        return true;
    }

    long const first_ordinal = completion_watermark_ + 1;
    size_type const offset = static_cast<size_type>(ordinal - first_ordinal);
    return offset < events_.size() && events_[offset]
           && events_[offset]->complete;
}

//---------------------------------------------------------------------------//
/*!
 * Whether transport is done and no additional hit batch can arrive.
 */
bool OpticalEventTable::is_transport_complete(long ordinal) const
{
    if (ordinal < 0)
    {
        return false;
    }
    if (ordinal <= completion_watermark_)
    {
        return true;
    }

    long const first_ordinal = completion_watermark_ + 1;
    size_type const offset = static_cast<size_type>(ordinal - first_ordinal);
    return offset < events_.size() && events_[offset]
           && this->transport_complete(*events_[offset]);
}

//---------------------------------------------------------------------------//
/*!
 * Get a registered, unresolved event.
 */
auto OpticalEventTable::event(long ordinal) -> EventState&
{
    CELER_VALIDATE(
        ordinal > completion_watermark_,
        << "optical event " << ordinal << " is not registered and unresolved");
    long const first_ordinal = completion_watermark_ + 1;
    size_type const offset = static_cast<size_type>(ordinal - first_ordinal);
    CELER_VALIDATE(offset < events_.size() && events_[offset],
                   << "optical event " << ordinal << " is not registered");
    CELER_VALIDATE(!events_[offset]->complete,
                   << "optical event " << ordinal << " is already complete");
    return *events_[offset];
}

//---------------------------------------------------------------------------//
/*!
 * Get a lane state.
 */
auto OpticalEventTable::lane(LaneId lane_id) -> LaneState&
{
    CELER_VALIDATE(lane_id && lane_id < lanes_.size(),
                   << "invalid optical event lane " << lane_id);
    return lanes_[*lane_id];
}

//---------------------------------------------------------------------------//
/*!
 * Get a lane state.
 */
auto OpticalEventTable::lane(LaneId lane_id) const -> LaneState const&
{
    CELER_VALIDATE(lane_id && lane_id < lanes_.size(),
                   << "invalid optical event lane " << lane_id);
    return lanes_[*lane_id];
}

//---------------------------------------------------------------------------//
/*!
 * Whether the lane's latest census clears an event.
 */
bool OpticalEventTable::census_clears(EventState const& event_state) const
{
    auto const& lane_state = this->lane(event_state.lane);
    return lane_state.census_valid
           && lane_state.census_epoch >= event_state.last_append_epoch
           && (!lane_state.min_live_ordinal
               || event_state.ordinal < *lane_state.min_live_ordinal);
}

//---------------------------------------------------------------------------//
/*!
 * Whether all non-delivery completion conditions are satisfied.
 */
bool OpticalEventTable::transport_complete(EventState const& event_state) const
{
    auto const& lane_state = this->lane(event_state.lane);
    return event_state.closed
           && event_state.absorbed_bursts == event_state.submitted_bursts
           && lane_state.generated_photons >= event_state.generation_target
           && this->census_clears(event_state);
}

//---------------------------------------------------------------------------//
/*!
 * Latch completion if an event satisfies all five conditions.
 */
void OpticalEventTable::update_event(EventState& event_state)
{
    if (event_state.complete)
    {
        return;
    }

    auto const& lane_state = this->lane(event_state.lane);
    if (this->transport_complete(event_state)
        && lane_state.delivered_through >= event_state.ordinal)
    {
        event_state.complete = true;
        CELER_ASSERT(unresolved_count_ > 0);
        --unresolved_count_;
    }
}

//---------------------------------------------------------------------------//
/*!
 * Reevaluate every unresolved event assigned to a lane.
 */
void OpticalEventTable::update_lane(LaneId lane_id)
{
    for (auto& event_state : events_)
    {
        if (event_state && !event_state->complete
            && event_state->lane == lane_id)
        {
            this->update_event(*event_state);
        }
    }
    this->advance_watermark();
}

//---------------------------------------------------------------------------//
/*!
 * Advance the ordered watermark without blocking per-event completion.
 */
void OpticalEventTable::advance_watermark()
{
    while (!events_.empty() && events_.front() && events_.front()->complete)
    {
        ++completion_watermark_;
        events_.pop_front();
    }
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
