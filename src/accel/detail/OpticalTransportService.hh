//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file accel/detail/OpticalTransportService.hh
//---------------------------------------------------------------------------//
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_set>
#include <vector>

#include "corecel/Types.hh"

#include "OpticalEventTable.hh"
#include "OpticalTransportLane.hh"

namespace celeritas
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Coordinate process-wide optical transport over single-owner lanes.
 *
 * The service owns all shared synchronization. Producer tokens are
 * individually single-owner and must not outlive the service. Hit callbacks
 * are globally serialized and run on the thread that calls pump or
 * drain_and_stop; callbacks must not reenter pump.
 */
class OpticalTransportService
{
  private:
    struct SharedState;

  public:
    //!@{
    //! \name Type aliases
    using LaneId = OpticalEventTable::LaneId;
    using HitCallback
        = std::function<void(long, std::vector<optical::DetectorHit> const&)>;
    using ActionTimeCallback
        = std::function<void(LaneId, OpticalTransportLaneInterface::MapStrDbl)>;
    using LaneFactory
        = std::function<std::unique_ptr<OpticalTransportLaneInterface>(LaneId)>;
    //!@}

    struct Options
    {
        size_type num_lanes{0};
        size_type unresolved_limit{0};
        size_type staged_bytes_limit{0};
        long base_ordinal{0};
        bool log_metrics{false};
    };

    struct PumpResult
    {
        bool pumped{false};
        bool complete{false};
        bool contended{false};
    };

    struct Statistics
    {
        size_type unresolved_events{0};
        size_type staged_bytes{0};
        size_type resident_events{0};
        size_type mailbox_hits{0};
        size_type max_unresolved_events{0};
        size_type max_staged_bytes{0};
        size_type max_resident_events{0};
        size_type max_mailbox_hits{0};
        size_type event_admission_waits{0};
        size_type staged_bytes_waits{0};
        size_type pump_calls{0};
        size_type active_producers{0};
        long completion_watermark{-1};
        bool stopped{false};
    };

    //----------------------------------------------------------------------//
    /*!
     * Move-only registration and submission handle for one producer.
     */
    class ProducerToken
    {
      public:
        ProducerToken(ProducerToken const&) = delete;
        ProducerToken& operator=(ProducerToken const&) = delete;
        ProducerToken(ProducerToken&&) noexcept;
        ProducerToken& operator=(ProducerToken&&) noexcept;
        ~ProducerToken();

        // Register an event and return its deterministic lane
        LaneId register_event(long ordinal);

        // Submit a bounded host-side burst
        void submit_burst(
            long ordinal, size_type num_photons, size_type size_bytes);

        // Submit real optical generator records, charging their host bytes
        void submit_burst(long ordinal,
                          std::vector<optical::GeneratorDistributionData>,
                          size_type num_photons);

        // Explicitly close an event against future submissions
        void close_event(long ordinal);

        // Pump and wait until an owned, closed event is complete, then release
        // it
        void wait_until_complete(long ordinal);

        // Query an owned event's completion state
        bool is_complete(long ordinal) const;

      private:
        friend class OpticalTransportService;

        explicit ProducerToken(std::shared_ptr<SharedState> state);
        void check_owned(long ordinal) const;
        void check_open(long ordinal) const;
        void release() noexcept;

        std::shared_ptr<SharedState> state_;
        std::unordered_set<long> owned_events_;
        std::unordered_set<long> open_events_;
    };

  public:
    // Construct the standalone shell with one injected implementation per lane
    OpticalTransportService(Options options,
                            LaneFactory make_lane,
                            HitCallback hit_callback = {},
                            ActionTimeCallback action_time_callback = {});
    ~OpticalTransportService();

    OpticalTransportService(OpticalTransportService const&) = delete;
    OpticalTransportService& operator=(OpticalTransportService const&) = delete;

    // Register one producer lifetime with the service
    ProducerToken make_producer();

    // Pump the target event's lane on the calling thread
    PumpResult pump(long ordinal);

    // Try the target event's lane without blocking on callback delivery
    PumpResult try_pump(long ordinal);

    // Try one lane and deliver any results already ready for callbacks
    PumpResult try_pump();

    // Query per-event completion
    bool is_complete(long ordinal) const;

    // Pump and wait until a registered, closed event is complete
    void wait_until_complete(long ordinal);

    // Drain every closed event, stop all lanes, and join their threads
    void drain_and_stop();

    // Snapshot bounded-memory and backpressure counters
    Statistics statistics() const;

  private:
    static void check_service(std::shared_ptr<SharedState> const& state);
    static LaneId
    register_event(std::shared_ptr<SharedState> const& state, long ordinal);
    static void submit_burst(std::shared_ptr<SharedState> const& state,
                             OpticalTransportBurst burst);
    static void
    close_event(std::shared_ptr<SharedState> const& state, long ordinal);
    static PumpResult pump(std::shared_ptr<SharedState> const& state,
                           std::optional<long> ordinal,
                           bool try_lock);
    static bool
    is_complete(std::shared_ptr<SharedState> const& state, long ordinal);
    static void wait_until_complete(std::shared_ptr<SharedState> const& state,
                                    long ordinal);
    static void
    release_producer(std::shared_ptr<SharedState> const& state,
                     std::unordered_set<long> const& open_events) noexcept;

    void lane_loop(LaneId lane);
    void stop_and_join() noexcept;

    std::shared_ptr<SharedState> state_;
    std::vector<std::unique_ptr<OpticalTransportLaneInterface>> lanes_;
    std::vector<std::thread> workers_;
    std::mutex lifecycle_mutex_;
};

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace celeritas
