//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/OpticalUtils.test.cc
//---------------------------------------------------------------------------//
#include "celeritas/optical/detail/OpticalUtils.hh"

#include <memory>
#include <numeric>
#include <vector>

#include "corecel/Types.hh"
#include "corecel/cont/Span.hh"
#include "corecel/data/CollectionAlgorithms.hh"
#include "corecel/data/CollectionBuilder.hh"
#include "corecel/data/Ref.hh"
#include "corecel/math/Algorithms.hh"
#include "celeritas/optical/action/detail/TrackInitAlgorithms.hh"
#include "celeritas/optical/gen/detail/GeneratorAlgorithms.hh"
#include "celeritas/optical/detail/EventCensus.hh"

#include "celeritas_test.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
// EVENT CENSUS ARITHMETIC
//---------------------------------------------------------------------------//
/*!
 * The census reduces the smallest live event ordinal RELATIVE to a base,
 * modulo the ring the ordinal is encoded in. That is what lets a streaming
 * driver retire events while later ones are still in flight, and it has to
 * keep ordering across the wrap -- which is the whole reason it is not a
 * plain minimum.
 */
class EventCensusArithmeticTest : public Test
{
  protected:
    using Counters = CoreStateCounters;

    //! Reduce a set of event ordinals the way the census kernels do
    size_type reduce(size_type base, std::vector<size_type> const& events)
    {
        Counters c;
        c.event_census_base = base;
        c.min_live_event_rel = optical::event_ring;
        for (size_type ev : events)
        {
            auto primary = id_cast<PrimaryId>(ev << optical::event_shift);
            optical::detail::census_event(&c, primary);
        }
        return c.min_live_event_rel;
    }
};

TEST_F(EventCensusArithmeticTest, empty_reads_as_nothing_live)
{
    EXPECT_EQ(optical::event_ring, this->reduce(0, {}));
    EXPECT_EQ(optical::event_ring, this->reduce(1234, {}));
}

TEST_F(EventCensusArithmeticTest, picks_the_oldest)
{
    // Relative to the base, so the answer is an offset from it
    EXPECT_EQ(0, this->reduce(10, {10, 11, 12}));
    EXPECT_EQ(1, this->reduce(10, {13, 11, 12}));
    EXPECT_EQ(5, this->reduce(10, {15}));
    // Order of contribution must not matter
    EXPECT_EQ(1, this->reduce(10, {12, 11, 13}));
    EXPECT_EQ(1, this->reduce(10, {13, 12, 11}));
}

TEST_F(EventCensusArithmeticTest, orders_correctly_across_the_wrap)
{
    // Base near the top of the ring with events that have wrapped past zero:
    // a plain minimum would call the wrapped ordinals oldest and retire
    // events whose photons are still in flight.
    size_type const ring = optical::event_ring;
    size_type const base = ring - 2;
    // Events base, base+1, then 0 and 1 (wrapped) -- base is still oldest
    EXPECT_EQ(0, this->reduce(base, {base, base + 1, 0, 1}));
    // With base itself retired, the oldest live one is base+1 (offset 1)
    EXPECT_EQ(1, this->reduce(base, {base + 1, 0, 1}));
    // Only wrapped events remain: offsets 2 and 3 past the base
    EXPECT_EQ(2, this->reduce(base, {0, 1}));
}

TEST_F(EventCensusArithmeticTest, unset_primary_counts_as_event_zero)
{
    // A photon with no primary id belongs to no event; treated as ordinal 0
    // so it can never make an event look retired while it is alive.
    Counters c;
    c.event_census_base = 0;
    c.min_live_event_rel = optical::event_ring;
    optical::detail::census_event(&c, PrimaryId{});
    EXPECT_EQ(0, c.min_live_event_rel);
}

//---------------------------------------------------------------------------//
template<class T, MemSpace M>
using StateVal = StateCollection<T, Ownership::value, M>;
template<class T, MemSpace M>
using StateRef = StateCollection<T, Ownership::reference, M>;

template<MemSpace M>
std::vector<int> locate_vacancies(std::vector<TrackStatus> const& input)
{
    if constexpr (M == MemSpace::device)
    {
        device().create_streams(1);
    }

    StateVal<TrackStatus, MemSpace::host> host_status;
    make_builder(&host_status).insert_back(input.begin(), input.end());
    StateVal<TrackStatus, M> status(host_status);

    StateVal<TrackSlotId, M> vacancies;
    resize(&vacancies, status.size());

    StateRef<TrackStatus, M> status_ref(status);
    StateRef<TrackSlotId, M> vacancies_ref(vacancies);
    optical::detail::VacancyScratch scratch;
    if constexpr (M == MemSpace::device)
    {
        scratch.result = DeviceVector<size_type>(1, StreamId{0});
    }
    size_type num_vacancies = optical::detail::copy_if_vacant(
        status_ref, vacancies_ref, &scratch, StreamId{0});

    auto host_vacancies = copy_to_host(vacancies);

    std::vector<int> result;
    for (auto tid : range(TrackSlotId{num_vacancies}))
    {
        result.push_back(static_cast<int>(host_vacancies[tid].unchecked_get()));
    }
    return result;
}

//---------------------------------------------------------------------------//
// TESTS
//---------------------------------------------------------------------------//

TEST(OpticalUtilsTest, find_distribution_index)
{
    using optical::detail::find_distribution_index;

    size_type num_threads = 8;
    std::vector<size_type> vacancies = {1, 2, 4, 6, 7};

    // Number of photons to generate from each distribution
    std::vector<size_type> distributions = {1, 1, 5, 2, 5, 8, 1, 6, 7, 7};

    // Calculate the inclusive prefix sum of the number of photons
    std::vector<size_type> counts(distributions.size());
    std::partial_sum(
        distributions.begin(), distributions.end(), counts.begin());
    {
        static unsigned int const expected_counts[]
            = {1u, 2u, 7u, 9u, 14u, 22u, 23u, 29u, 36u, 43u};
        EXPECT_VEC_EQ(expected_counts, counts);
    }

    auto fill_vacancies = [&]() {
        std::vector<int> result(num_threads, -1);

        // Find the index of the first distribution that has a nonzero number
        // of primaries left to generate
        auto start = celeritas::upper_bound(counts.begin(), counts.end(), 0_sz);

        size_type offset = start - counts.begin();
        Span<size_type> span_counts{start, counts.end()};

        for (auto thread_idx : range(vacancies.size()))
        {
            // In the vacsnt track slot, store the index of the distribution
            // that will generate the track
            result[vacancies[thread_idx]]
                = offset + find_distribution_index(span_counts, thread_idx);
        }
        return result;
    };

    {
        auto result = fill_vacancies();
        static int const expected_result[] = {-1, 0, 1, -1, 2, -1, 2, 2};
        EXPECT_VEC_EQ(expected_result, result);
    }

    size_type num_gen = vacancies.size();
    for (auto thread_idx : range(counts.size()))
    {
        // Update the cumulative sum of the number of photons per distribution
        if (counts[thread_idx] < num_gen)
        {
            counts[thread_idx] = 0;
        }
        else
        {
            counts[thread_idx] -= num_gen;
        }
    }
    {
        static unsigned int const expected_counts[]
            = {0u, 0u, 2u, 4u, 9u, 17u, 18u, 24u, 31u, 38u};
        EXPECT_VEC_EQ(expected_counts, counts);
    }
    {
        auto result = fill_vacancies();
        static int const expected_result[] = {-1, 2, 2, -1, 3, -1, 3, 4};
        EXPECT_VEC_EQ(expected_result, result);
    }
}

TEST(OpticalUtilsTest, copy_if_vacant_host)
{
    using TS = TrackStatus;

    std::vector<TrackStatus> status = {
        TS::alive,
        TS::killed,
        TS::alive,
        TS::alive,
        TS::initializing,
        TS::errored,
        TS::alive,
        TS::killed,
    };
    auto vacancies = locate_vacancies<MemSpace::host>(status);

    EXPECT_EQ(4, vacancies.size());
    static int const expected_vacancies[] = {1, 4, 5, 7};
    EXPECT_VEC_EQ(expected_vacancies, vacancies);
}

TEST(OpticalUtilsTest, TEST_IF_CELER_DEVICE(copy_if_vacant_device))
{
    using TS = TrackStatus;

    std::vector<TrackStatus> status = {
        TS::alive,
        TS::alive,
        TS::initializing,
        TS::initializing,
        TS::killed,
        TS::killed,
        TS::alive,
        TS::alive,
    };
    auto vacancies = locate_vacancies<MemSpace::device>(status);

    EXPECT_EQ(4, vacancies.size());
    static int const expected_vacancies[] = {2, 3, 4, 5};
    EXPECT_VEC_EQ(expected_vacancies, vacancies);
}

/*!
 * The persistent scratch must serve repeated selections with changing
 * vacancy patterns: correct sorted slot ids each time (the selection is
 * stable, so ascending order is the contract), the same count, and no
 * reallocation -- the scratch pointers grown by the first call must not
 * move afterward.
 */
TEST(OpticalUtilsTest, TEST_IF_CELER_DEVICE(copy_if_vacant_scratch_persists))
{
    using TS = TrackStatus;

    device().create_streams(1);
    StreamId stream{0};
    constexpr size_type num_slots = 64;

    // Irregular, pass-dependent vacancy patterns over the same slots
    auto make_status = [](int pass) {
        std::vector<TrackStatus> status(num_slots, TS::alive);
        for (size_type i = 0; i < num_slots; ++i)
        {
            if ((pass == 0 && (i % 3 == 0 || i % 7 == 0))
                || (pass == 1 && i % 5 == 2) || (pass == 2 && i >= 60))
            {
                status[i] = (i % 2 ? TS::killed : TS::initializing);
            }
        }
        return status;
    };

    StateVal<TrackSlotId, MemSpace::device> vacancies;
    resize(&vacancies, num_slots);
    StateRef<TrackSlotId, MemSpace::device> vacancies_ref(vacancies);

    optical::detail::VacancyScratch scratch;
    scratch.result = DeviceVector<size_type>(1, stream);

    size_type const* result_ptr = scratch.result.data();
    char const* temp_ptr = nullptr;
    unsigned char const* flags_ptr = nullptr;

    for (int pass = 0; pass < 3; ++pass)
    {
        auto status = make_status(pass);
        StateVal<TrackStatus, MemSpace::host> host_status;
        make_builder(&host_status).insert_back(status.begin(), status.end());
        StateVal<TrackStatus, MemSpace::device> dev_status(host_status);
        StateRef<TrackStatus, MemSpace::device> status_ref(dev_status);

        std::vector<size_type> expected;
        for (size_type i = 0; i < num_slots; ++i)
        {
            if (status[i] != TS::alive)
            {
                expected.push_back(i);
            }
        }
        ASSERT_GT(expected.size(), 0u) << "pass " << pass;

        size_type num_vacancies = optical::detail::copy_if_vacant(
            status_ref, vacancies_ref, &scratch, stream);
        EXPECT_EQ(expected.size(), num_vacancies) << "pass " << pass;

        auto host_vacancies = copy_to_host(vacancies);
        for (size_type i = 0; i < expected.size(); ++i)
        {
            EXPECT_EQ(expected[i],
                      host_vacancies[TrackSlotId{i}].unchecked_get())
                << "pass " << pass << " index " << i;
        }

        if (pass == 0)
        {
            // Whatever the first call grew is what every later call reuses
            temp_ptr = scratch.temp.size() ? scratch.temp.data() : nullptr;
            flags_ptr = scratch.flags.size() ? scratch.flags.data() : nullptr;
        }
        else
        {
            EXPECT_EQ(result_ptr, scratch.result.data()) << "pass " << pass;
            EXPECT_EQ(temp_ptr,
                      scratch.temp.size() ? scratch.temp.data() : nullptr)
                << "pass " << pass;
            EXPECT_EQ(flags_ptr,
                      scratch.flags.size() ? scratch.flags.data() : nullptr)
                << "pass " << pass;
        }
    }
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
