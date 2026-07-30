//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/CoreState.hh
//---------------------------------------------------------------------------//
#pragma once

#include <functional>

#include "corecel/cont/Span.hh"
#include "corecel/data/AuxInterface.hh"
#include "corecel/data/AuxStateVec.hh"
#include "corecel/data/ObserverPtr.hh"
#include "corecel/data/StateDataStore.hh"
#include "corecel/random/params/RngParamsFwd.hh"
#include "celeritas/Types.hh"
#include "celeritas/phys/GeneratorCounters.hh"
#include "celeritas/track/CoreStateCounters.hh"

#include "CoreTrackData.hh"
#include "DetectorData.hh"
#include "TrackInitializer.hh"

namespace celeritas
{
namespace optical
{
class CoreParams;

//---------------------------------------------------------------------------//
/*!
 * Interface class for optical state data.
 *
 * This inherits from the "aux state" interface to allow stream-local storage
 * with the optical offload data.
 */
class CoreStateInterface : public AuxStateInterface
{
  public:
    //!@{
    //! \name Type aliases
    using size_type = TrackSlotId::size_type;
    //!@}

  public:
    // Support polymorphic deletion
    ~CoreStateInterface() override;

    //! Thread/stream ID
    virtual StreamId stream_id() const = 0;

    //! Synchronize and copy track initialization counters from device to host
    //! For host-only code, this replaces the old counters() function
    [[nodiscard]] virtual CoreStateCounters sync_get_counters() const = 0;

    //! Synchronize and copy track initialization counters from host to device
    //! For host-only code, this replaces the old counters() function
    //! since we return a CoreStateCounters object instead of a reference
    virtual void sync_put_counters(CoreStateCounters const&) = 0;

    //! Reseed the RNGs at the start of an event for reproducibility
    virtual void reseed(std::shared_ptr<RngParams const>, UniqueEventId) = 0;

    //! Reset all track slots and counters (recovery after an aborted loop)
    virtual void reset() = 0;

    //! Number of track slots
    virtual size_type size() const = 0;

    //! Number of leading track slots holding a live track
    virtual size_type active_size() const = 0;

    //! Record how many leading slots hold a live track
    virtual void active_size(size_type) = 0;

    // Inject optical primaries
    virtual void
    insert_primaries(Span<TrackInitializer const> host_primaries) = 0;

  protected:
    CoreStateInterface() = default;

    CELER_DEFAULT_COPY_MOVE(CoreStateInterface);
};

//---------------------------------------------------------------------------//
/*!
 * Manage the optical state counters and auxiliary data.
 */
class CoreStateBase : public CoreStateInterface
{
  public:
    //!@{
    //! \name Type aliases
    using SPAuxStateVec = std::shared_ptr<AuxStateVec>;
    using HitSink = std::function<void(Span<DetectorHit const>)>;
    //!@}

  public:
    //! Optical loop statistics
    CounterAccumStats const& accum() const { return accum_; }

    //! Optical loop statistics
    CounterAccumStats& accum() { return accum_; }

    //! Per-stream hit redirection: when set, the detector action delivers
    //! hits here instead of the global callback. Used by the streaming
    //! driver so hits cross to the producer thread instead of being
    //! delivered on the transport thread.
    HitSink const& hit_sink() const { return hit_sink_; }

    //! Set (or clear, with nullptr) the hit redirection
    void hit_sink(HitSink sink) { hit_sink_ = std::move(sink); }

    //! Cumulative num_hits value already delivered: the detector action
    //! skips the hit copy when the device count has not moved
    size_type last_hit_count() const { return last_hit_count_; }

    //! Record the delivered hit count
    void last_hit_count(size_type n) { last_hit_count_ = n; }

    //// AUXILIARY DATA ////

    //! Access auxiliary core state data
    SPAuxStateVec const& aux() const { return aux_state_; }

    //! Access auxiliary core state data (mutable)
    SPAuxStateVec& aux() { return aux_state_; }

  protected:
    CoreStateBase() = default;

    // Anchor vtable
    ~CoreStateBase() override;

  private:
    //! Counts accumulated over the event for diagnostics
    CounterAccumStats accum_;

    // Auxiliary data owned by the core state
    SPAuxStateVec aux_state_;

    // Streaming hit redirection (unset outside streaming mode)
    HitSink hit_sink_;

    // Hits delivered so far (see num_hits counter)
    size_type last_hit_count_{0};
};

//---------------------------------------------------------------------------//
/*!
 * Store all state data for a single thread.
 *
 * When the state lives on the device, we maintain a separate copy of the
 * device "ref" in device memory: otherwise we'd have to copy the entire state
 * in launch arguments and access it through constant memory.
 *
 * \todo Encapsulate all the action management accessors in a helper class.
 */
template<MemSpace M>
class CoreState final : public CoreStateBase
{
  public:
    //!@{
    //! \name Type aliases
    template<template<Ownership, MemSpace> class S>
    using StateRef = S<Ownership::reference, M>;

    using Ref = StateRef<CoreStateData>;
    using Ptr = ObserverPtr<Ref, M>;
    //!@}

  public:
    // Construct from CoreParams
    CoreState(CoreParams const& params,
              StreamId stream_id,
              size_type num_track_slots);

    // Default destructor
    ~CoreState() final;

    //! Thread/stream ID
    StreamId stream_id() const final { return this->ref().stream_id; }

    //! Number of track slots
    size_type size() const final { return states_.size(); }

    //! Number of leading track slots holding a live track. The
    //! thread-to-slot map is kept partitioned so an action can launch over
    //! these alone instead of every slot.
    size_type active_size() const final { return active_size_; }

    //! Record how many leading slots hold a live track
    void active_size(size_type n) final
    {
        CELER_EXPECT(n <= this->size());
        active_size_ = n;
    }

    //! Synchronize and copy track initialization counters from device to host
    [[nodiscard]] CoreStateCounters sync_get_counters() const final;

    //! Synchronize and copy track initialization counters from host to device
    //! For host-only code, this copies the local CoreStateCounters back to the
    //! class, since sync_get_counters() doesn't return a reference
    void sync_put_counters(CoreStateCounters const&) final;

    // Whether the state is being transported with no active particles
    bool warming_up() const;

    //// CORE DATA ////

    //! Get a reference to the mutable state data
    Ref& ref() { return states_.ref(); }

    //! Get a reference to the mutable state data
    Ref const& ref() const { return states_.ref(); }

    //! Get a native-memspace pointer to the mutable state data
    Ptr ptr() { return ptr_; }

    // Reset the data for a new step
    void reset() final;

    // Reseed the RNGs at the start of an event for reproducibility
    void reseed(std::shared_ptr<RngParams const>, UniqueEventId) final;

    // Inject primaries to be turned into TrackInitializers
    void insert_primaries(Span<TrackInitializer const> host_primaries) final;

  private:
    // State data
    StateDataStore<CoreStateData, M> states_;

    // Copy of state ref in device memory, if M == MemSpace::device
    DeviceVector<Ref> device_ref_vec_;

    // Native pointer to ref or
    Ptr ptr_;

    // Leading slots holding a live track, maintained by the partition
    size_type active_size_{0};
};

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
