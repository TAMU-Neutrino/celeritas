//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/Transporter.cc
//---------------------------------------------------------------------------//
#include "Transporter.hh"

#include <algorithm>

#include "detail/PartitionTracks.hh"
#include "detail/SortTracks.hh"

#include <cstdlib>

#include <utility>

#include "corecel/io/Logger.hh"
#include "corecel/sys/ScopedProfiling.hh"
#include "corecel/sys/Stopwatch.hh"
#include "celeritas/phys/GeneratorRegistry.hh"  // IWYU pragma: keep

#include "CoreParams.hh"
#include "CoreState.hh"
#include "SimParams.hh"  // IWYU pragma: keep
#include "detail/EventCensus.hh"

namespace celeritas
{
namespace optical
{
//---------------------------------------------------------------------------//
/*!
 * Construct with problem parameters.
 */
Transporter::Transporter(Input&& inp) : input_(std::move(inp))
{
    CELER_EXPECT(input_.params);

    actions_ = std::make_shared<ActionGroupsT>(*this->params()->action_reg());
}

//---------------------------------------------------------------------------//
/*!
 * Transport all pending optical tracks on the host.
 */
void Transporter::operator()(CoreStateBase& state) const
{
    if (auto* s = dynamic_cast<CoreStateHost*>(&state))
    {
        return this->transport_impl(*s);
    }
    else if (auto* s = dynamic_cast<CoreStateDevice*>(&state))
    {
        return this->transport_impl(*s);
    }
    CELER_ASSERT_UNREACHABLE();
}

//---------------------------------------------------------------------------//
/*!
 * Run a single step iteration of the optical loop.
 *
 * This is the body of the transport loop, shared by the flush-to-drain
 * operator() and the streaming driver. The iteration ordinal sequences the
 * periodic full compaction pass.
 */
template<MemSpace M>
CoreStateCounters Transporter::step_once_impl(CoreState<M>& state,
                                              size_type iter_ordinal,
                                              size_type census_base) const
{
    // Store a pointer to aux data for timing results
    std::vector<double>* accum_time = nullptr;
    if (input_.action_times)
    {
        accum_time = &input_.action_times->state(*state.aux()).accum_time;
    }

    // Opt-in until the 1% shift it produces on a 10-event sample is shown
    // to be statistical rather than a missed track: with it off the loop
    // launches over every slot, exactly as before.
    static bool const compact = std::getenv("CELER_TRACK_COMPACT") != nullptr;
    // Group the threads that will run by volume so a warp's lanes take the
    // same path through the solid tree instead of serialising over several.
    // Opt-in: it costs the coalescing that thread i -> slot i gives today,
    // and which of the two wins is a property of the geometry.
    static bool const sort_tracks = std::getenv("CELER_TRACK_SORT") != nullptr;
    static size_type const full_partition_period = [] {
        if (char const* s = std::getenv("CELER_TRACK_COMPACT_PERIOD"))
        {
            return static_cast<size_type>(std::max(1, std::atoi(s)));
        }
        return size_type{16};
    }();

    ScopedProfiling profile_this{"step"};
    Stopwatch get_step_time;

    bool const take_census
        = input_.census_period > 0 && (iter_ordinal % input_.census_period == 0);
    if (take_census)
    {
        // Open the census window: clear the running minimum and publish the
        // base the reduction is relative to. Contributions land through the
        // rest of this iteration -- pending re-emission records from the
        // generator, live tracks at the end -- and are read below.
        auto c = state.sync_get_counters();
        c.min_live_event_rel = event_ring;
        c.event_census_base = census_base;
        state.sync_put_counters(c);
    }

    // Gather the live tracks at the front of the thread-to-slot map so
    // that the actions after the pre-step launch over them alone. The
    // optical loop is tail-dominated -- a handful of photons diffusing
    // in a wavelength shifter keep it running long after the rest have
    // died -- so most of each launch would otherwise be empty slots.
    //
    // A full pass costs one sweep of the whole capacity, which is what
    // stops a large track state from paying for itself. Newly filled
    // slots can be anywhere, so a full pass is needed to pick them up,
    // but only periodically: in between, sweeping the active prefix
    // alone is enough to drop the tracks that just died. A track sitting
    // outside the prefix is not lost, only left for the next full pass,
    // since it stays alive and the loop runs until nothing is.
    if (compact)
    {
        bool const full = (iter_ordinal % full_partition_period == 0);
        size_type const num_threads
            = full ? state.size() : state.active_size();
        state.active_size(detail::partition_alive(state.ref(), num_threads));
    }
    else
    {
        state.active_size(state.size());
    }

    if (sort_tracks)
    {
        detail::sort_by_volume(state.ref(), state.active_size());
    }

    // Loop through actions
    for (auto const& action : actions_->step())
    {
        ScopedProfiling profile_this{action->label()};
        Stopwatch get_action_time;
        action->step(*this->params(), state);
        if (accum_time)
        {
            if (M == MemSpace::device)
            {
                device().stream(state.stream_id()).sync();
            }
            (*accum_time)[action->action_id().get()] += get_action_time();
        }
    }

    // Close the census window with the live tracks. The generator folded in
    // its pending re-emission records earlier in this same iteration, so the
    // minimum now covers every place a photon of a given event can be.
    if (take_census)
    {
        detail::census_tracks(state.ref(), state.active_size());
    }

    // Retrieve the counters updated on the device during the iteration. The
    // step instruments cover this synchronization, as they did when the
    // read lived in the loop.
    auto counters = state.sync_get_counters();

    // Record the step time
    if (input_.step_times)
    {
        if (M == MemSpace::device)
        {
            device().stream(state.stream_id()).sync();
        }
        auto& step_times = input_.step_times->state(*state.aux()).time;
        step_times.push_back(get_step_time());
    }

    return counters;
}

//---------------------------------------------------------------------------//
/*!
 * Run a single step iteration (streaming driver building block).
 */
CoreStateCounters
Transporter::step_once(CoreStateBase& state,
                       size_type iter_ordinal,
                       size_type census_base) const
{
    if (auto* s = dynamic_cast<CoreStateHost*>(&state))
    {
        return this->step_once_impl(*s, iter_ordinal, census_base);
    }
    else if (auto* s = dynamic_cast<CoreStateDevice*>(&state))
    {
        return this->step_once_impl(*s, iter_ordinal, census_base);
    }
    CELER_ASSERT_UNREACHABLE();
}

//---------------------------------------------------------------------------//
/*!
 * Transport all pending optical tracks.
 */
template<MemSpace M>
void Transporter::transport_impl(CoreState<M>& state) const
{
    CELER_EXPECT(state.aux());

    CELER_LOG_LOCAL(status) << "Transporting on " << to_cstring(M);

    size_type num_step_iters{0};
    size_type num_steps{0};

    auto counters = state.sync_get_counters();

    static bool const trace_occupancy
        = std::getenv("CELER_DEBUG_OCCUPANCY") != nullptr;

    // Loop while photons are yet to be tracked. A distribution written by
    // the last live track in its final iteration is in neither count -- the
    // generator only folds it into num_pending on the next pass -- so the
    // fresh-record flag must also hold the loop open.
    while (counters.num_pending > 0 || counters.num_alive > 0
           || counters.num_dist_written > 0)
    {
        // Run the iteration; the returned counters were synchronized after
        // the last action of the step
        counters = this->step_once_impl(state, num_step_iters, 0);
        num_steps += counters.num_active;

        if (CELER_UNLIKELY(trace_occupancy && num_step_iters < 40))
        {
            // Per-iteration view of why the slots are empty: if pending
            // photons are waiting while vacancies go unused, the loop is
            // generator-starved rather than tail-limited.
            CELER_LOG_LOCAL(warning)
                << "[OCC] iter " << num_step_iters
                << " active " << counters.num_active
                << " alive " << counters.num_alive
                << " vacancies " << counters.num_vacancies
                << " initializers " << counters.num_initializers
                << " pending " << counters.num_pending;
        }

        if (CELER_UNLIKELY(
                ++num_step_iters == this->params()->sim()->max_step_iters()))
        {
            CELER_LOG_LOCAL(error)
                << "Exceeded step count of "
                << this->params()->sim()->max_step_iters()
                << ": aborting optical transport loop with "
                << counters.num_generated << " generated tracks, "
                << counters.num_active << " active tracks, "
                << counters.num_alive << " alive tracks, "
                << counters.num_vacancies << " vacancies, "
                << counters.num_pending << " queued, " << counters.num_cut
                << " cut, and " << counters.num_errored << " errored";
            // Add all untransported tracks to 'cut' counter
            counters.num_cut += counters.num_active + counters.num_pending;

            this->params()->gen_reg()->reset(*state.aux());
            state.reset();
            break;
        }
    }
    if (counters.num_cut > 0)
    {
        CELER_LOG_LOCAL(warning) << "Terminated " << counters.num_cut
                                 << " optical tracks due to cutoff/limits";
    }
    if (counters.num_errored > 0)
    {
        CELER_LOG_LOCAL(error) << "Terminated " << counters.num_errored
                               << " optical tracks that errored";
    }

    // Update statistics
    state.accum().steps += num_steps;
    state.accum().step_iters += num_step_iters;
    ++state.accum().flushes;

    if (std::getenv("CELER_DEBUG_OCCUPANCY"))
    {
        // Every optical action launches one thread per track SLOT, so the
        // fraction of slots holding a live track is the ceiling on what
        // launching over live tracks instead could recover.
        double const slots = static_cast<double>(state.size());
        double const iters = static_cast<double>(num_step_iters);
        CELER_LOG_LOCAL(warning)
            << "[OCCUPANCY] flush: " << num_step_iters << " step iterations, "
            << num_steps << " track-steps, " << state.size() << " slots, "
            << "mean occupancy "
            << (iters > 0 ? 100 * num_steps / (iters * slots) : 0.0) << "%";
    }

    // Accumulate cut/error counters from the last synchronized counters
    state.accum().num_cut += counters.num_cut;
    state.accum().num_errored += counters.num_errored;
}

//---------------------------------------------------------------------------//
/*!
 * Get the accumulated action times.
 */
auto Transporter::get_action_times(AuxStateVec const& aux) const -> MapStrDbl
{
    if (input_.action_times)
    {
        return input_.action_times->get_action_times(aux);
    }
    return {};
}

//---------------------------------------------------------------------------//
/*!
 * Get the recorded step times.
 */
auto Transporter::get_step_times(AuxStateVec const& aux) const -> VecDbl
{
    if (input_.step_times)
    {
        return input_.step_times->state(aux).time;
    }
    return {};
}

//---------------------------------------------------------------------------//
}  // namespace optical
}  // namespace celeritas
