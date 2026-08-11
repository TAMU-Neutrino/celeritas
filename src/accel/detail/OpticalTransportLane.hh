//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalTransportLane.hh
//! \sa OpticalTransportService.test.cc
//---------------------------------------------------------------------------//
#pragma once

#include <functional>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include "corecel/Types.hh"
#include "celeritas/optical/DetectorData.hh"
#include "celeritas/optical/gen/GeneratorData.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * One host-side burst submitted to an optical transport lane.
 */
struct OpticalTransportBurst
{
    long event{-1};
    size_type num_photons{0};
    size_type size_bytes{0};
    std::vector<optical::GeneratorDistributionData> records;
};

//---------------------------------------------------------------------------//
/*!
 * Hits from one event returned by an optical transport lane.
 */
struct OpticalTransportHitBatch
{
    long event{-1};
    std::vector<optical::DetectorHit> hits;
};

//---------------------------------------------------------------------------//
/*!
 * Generation, census, and hit progress published by a host-side lane.
 */
struct OpticalTransportLaneProgress
{
    size_type total_generated{0};
    std::vector<OpticalTransportHitBatch> hit_batches;
    bool census_fresh{false};
    std::optional<long> min_live_ordinal;
};

//---------------------------------------------------------------------------//
enum class OpticalTransportLaneCommandType
{
    burst,
    close
};

//---------------------------------------------------------------------------//
/*!
 * One service command passed to a lane runner.
 */
struct OpticalTransportLaneCommand
{
    OpticalTransportLaneCommandType type{
        OpticalTransportLaneCommandType::burst};
    OpticalTransportBurst burst;
};

//---------------------------------------------------------------------------//
/*!
 * Service callbacks used by a lane's single-owner run loop.
 */
struct OpticalTransportLaneControl
{
    enum class ReceiveStatus
    {
        idle,
        work,
        stop
    };

    using VecCommand = std::vector<OpticalTransportLaneCommand>;
    using Receive = std::function<ReceiveStatus(VecCommand&, bool)>;
    using CensusBase = std::function<long()>;
    using Publish = std::function<void(OpticalTransportLaneProgress)>;

    Receive receive;  //!< Take available commands; optionally block
    CensusBase census_base;  //!< First unresolved run-global ordinal
    Publish publish;  //!< Return generation, census, and hit progress
};

//---------------------------------------------------------------------------//
/*!
 * Interface driven by one service-owned host thread.
 *
 * Implementations are single-owner: c run executes on the lane thread and
 * never concurrently with another call on the instance. The default runner
 * dispatches commands through c transport and c close_event for synchronous
 * and fake implementations. Finalize runs once after the owner thread joins.
 */
class OpticalTransportLaneInterface
{
  public:
    virtual ~OpticalTransportLaneInterface() = default;

    // Absorb and transport one burst to the reported progress point
    virtual OpticalTransportLaneProgress
    transport(OpticalTransportBurst const& burst) = 0;

    // Finish an event after all of its bursts have been submitted
    virtual OpticalTransportLaneProgress close_event(long ordinal) = 0;

    // Run until the service asks this single-owner lane to stop
    virtual void run(OpticalTransportLaneControl&);

    // Emit lane-local finalization data after the owner thread has stopped
    virtual void finalize() {}
};

//---------------------------------------------------------------------------//
/*!
 * Run the synchronous command adapter used by simple and fake lanes.
 */
inline void
OpticalTransportLaneInterface::run(OpticalTransportLaneControl& control)
{
    while (true)
    {
        OpticalTransportLaneControl::VecCommand commands;
        auto const status = control.receive(commands, true);
        if (status == OpticalTransportLaneControl::ReceiveStatus::stop)
        {
            return;
        }
        if (status == OpticalTransportLaneControl::ReceiveStatus::idle)
        {
            continue;
        }

        OpticalTransportLaneProgress combined;
        for (auto const& command : commands)
        {
            OpticalTransportLaneProgress current;
            if (command.type == OpticalTransportLaneCommandType::burst)
            {
                current = this->transport(command.burst);
            }
            else
            {
                current = this->close_event(command.burst.event);
            }

            combined.total_generated = current.total_generated;
            combined.hit_batches.insert(
                combined.hit_batches.end(),
                std::make_move_iterator(current.hit_batches.begin()),
                std::make_move_iterator(current.hit_batches.end()));
            combined.census_fresh = current.census_fresh;
            combined.min_live_ordinal = current.min_live_ordinal;
        }
        control.publish(std::move(combined));
    }
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
