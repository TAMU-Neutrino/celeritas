//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/track/CoreStateCounters.hh
//---------------------------------------------------------------------------//
#pragma once

#include "corecel/Types.hh"

namespace celeritas
{
//---------------------------------------------------------------------------//
/*!
 * Counters for within-step track initialization and activity.
 *
 * When running device code, these counters are now updated on the device
 * throughout the step, so they are stored in TrackInitStateData. They need to
 * be synchronized between the host and device before and after the step to
 * maintain consistency.
 *
 * For all user \c StepActionOrder (TODO: this may change if we add a
 * "user_end"), all but the secondaries/alive
 * counts are for the current step iteration, and secondaries/alive values
 * are from the previous step iteration.
 *
 * \todo Drop the 'num' prefix since we know they're counters.
 */
struct CoreStateCounters
{
    //!@{
    //! \name Set when primaries are generated
    size_type num_pending{0};  //!< Number waiting to be generated
    size_type num_generated{0};  //!< Number of track initializers created
    //!@}
    //
    //!@{
    //! \name Updated during generation and initialization
    size_type num_initializers{0};  //!< Number of track initializers
    size_type num_vacancies{0};  //!< Number of empty track slots
    //!@}

    //!@{
    //! \name Set after tracks are initialized
    size_type num_active{0};  //!< Number of active tracks at start
    //!@}

    //!@{
    //! \name Set after secondaries are generated
    size_type num_secondaries{0};  //!< Number of secondaries produced
    size_type num_alive{0};  //!< Number of alive tracks at end
    //!@}

    //!@{
    //! \name Set by CUDA CUB when partitioning the tracks, unused by celeritas
    size_type num_neutral{0};  //!< Number of neutral tracks
    //!@}

    //!@{
    //! \name Atomically incremented during stepping
    size_type num_cut{0};  //!< Number of tracks killed by tracking cuts
    size_type num_errored{0};  //!< Number of tracks killed due to errors
    //!@}

    //!@{
    //! \name Atomically incremented during optical stepping
    //! Cumulative count of scored detector hits: lets the detector action
    //! skip the hit copy on iterations that scored none (the tail is mostly
    //! such iterations)
    size_type num_hits{0};
    //! WLS distributions written by tracks since the WLS generator last ran:
    //! lets the generator skip its compact/scan/fill pipeline when nothing
    //! is shifting
    size_type num_dist_written{0};
    //!@}

    //!@{
    //! \name Event census (streaming injection)
    //
    // Which events still have photons somewhere in the loop. Without this,
    // the only completion signal is the whole loop going empty, which under
    // continuous injection may never happen -- so nothing could ever be
    // released while the run is in progress.
    //
    // The census reduces, over both live tracks and pending wavelength-shift
    // records, the event ordinal RELATIVE to \c event_census_base, modulo the
    // encoding ring. Relative because the ordinal is carried inside the
    // primary id in a field that wraps: differences from a host-chosen base
    // order correctly as long as fewer than a ring's worth of events are in
    // flight at once, which backpressure guarantees. The host adds the base
    // back, and may then retire every event below the result whose photons
    // have all been generated.
    //
    // \c min_live_event_rel is reset to the ring size (meaning "nothing
    // live") before each census pass.
    size_type event_census_base{0};
    size_type min_live_event_rel{0};
    //!@}
};

//---------------------------------------------------------------------------//
/*!
 * Whether optical transport still has work.
 *
 * Photons pending generation, live tracks, or a distribution stored by a
 * track since the generator last ran. The last term matters when the storing
 * track died in the same iteration: the record reaches \c num_pending only on
 * the generator's next pass, so without it a blocking loop would exit -- and
 * a streaming consumer would park -- with the photon still queued.
 */
inline bool has_transportable_work(CoreStateCounters const& counters)
{
    return counters.num_pending > 0 || counters.num_alive > 0
           || counters.num_dist_written > 0;
}

//---------------------------------------------------------------------------//
}  // namespace celeritas
