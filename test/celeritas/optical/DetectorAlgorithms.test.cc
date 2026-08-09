//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/DetectorAlgorithms.test.cc
//---------------------------------------------------------------------------//
#include <algorithm>
#include <vector>

#include "corecel/data/CollectionBuilder.hh"
#include "corecel/data/Copier.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/Stream.hh"
#include "celeritas/optical/DetectorData.hh"
#include "celeritas/optical/action/detail/DetectorAlgorithms.hh"
#include "celeritas/optical/action/detail/DetectorHitBuffer.hh"

#include "celeritas_test.hh"

namespace celeritas
{
namespace test
{
//---------------------------------------------------------------------------//
/*!
 * The device compaction must deliver exactly the valid hits, whole-struct,
 * in ascending slot order -- the order the host-side filter it replaces
 * has always produced. No geometry, no params: the state buffer is built
 * by hand with a recognizable payload per slot.
 */
TEST(DetectorAlgorithmsTest, TEST_IF_CELER_DEVICE(copy_if_hit_device))
{
    using optical::DetectorHit;
    using Energy = DetectorHit::Energy;

    device().create_streams(1);
    StreamId stream{0};

    constexpr size_type num_slots = 64;

    // Valid hits on an irregular pattern of slots, each with fields that
    // encode its slot so misordering or truncation is visible
    optical::DetectorStateData<Ownership::value, MemSpace::host> host_state;
    resize(&host_state, num_slots);
    auto host_hits = host_state.detector_hits[
        AllItems<DetectorHit, MemSpace::host>{}];
    std::vector<DetectorHit> expected;
    for (unsigned int i = 0; i < num_slots; ++i)
    {
        DetectorHit hit;
        hit.energy = Energy{0};
        if (i % 3 == 0 || i % 7 == 0)
        {
            hit.detector = DetectorId{i % 4};
            hit.primary = PrimaryId{i + 1};
            hit.energy = Energy{1e-6 * static_cast<real_type>(i + 1)};
            hit.time = 0.5 * static_cast<real_type>(i);
            hit.position = Real3{static_cast<real_type>(i), -1.0, 2.0};
            hit.volume_instance = VolumeInstanceId{i};
            expected.push_back(hit);
        }
        host_hits[i] = hit;
    }
    ASSERT_LT(expected.size(), num_slots);
    ASSERT_GT(expected.size(), 0);

    // Mirror the state on device and build the persistent buffers the way
    // DetectorAction::create_state does
    optical::DetectorStateData<Ownership::value, MemSpace::device> dev_state;
    dev_state = host_state;
    optical::detail::DetectorStateRef<MemSpace::device> dev_ref;
    dev_ref = dev_state;

    optical::detail::DetectorHitBuffer buf;
    buf.compact = DeviceVector<DetectorHit>(num_slots, stream);
    buf.result = DeviceVector<size_type>(1, stream);
    buf.host.resize(num_slots);

    // Run twice: the second call must reuse the scratch sized by the first
    char const* temp_after_first = nullptr;
    for (int pass = 0; pass < 2; ++pass)
    {
        optical::detail::copy_if_hit(
            dev_ref, &buf, expected.size(), stream);
        if (pass == 0)
        {
            temp_after_first = buf.temp.size() ? buf.temp.data() : nullptr;
        }

        std::vector<DetectorHit> actual(expected.size());
        Copier<DetectorHit, MemSpace::host> copy_back{make_span(actual),
                                                      stream};
        copy_back(MemSpace::device,
                  {buf.compact.data(), expected.size()});
        device().stream(stream).sync();

        for (size_type i = 0; i < expected.size(); ++i)
        {
            EXPECT_EQ(expected[i].detector, actual[i].detector)
                << "hit " << i << " pass " << pass;
            EXPECT_EQ(expected[i].primary, actual[i].primary);
            EXPECT_EQ(expected[i].energy.value(), actual[i].energy.value());
            EXPECT_EQ(expected[i].time, actual[i].time);
            EXPECT_EQ(expected[i].position, actual[i].position);
            EXPECT_EQ(expected[i].volume_instance, actual[i].volume_instance);
        }

        if (pass == 1 && temp_after_first)
        {
            // Persistence: the scratch pointer did not move between calls
            EXPECT_EQ(temp_after_first, buf.temp.data());
        }
    }
}

//---------------------------------------------------------------------------//
}  // namespace test
}  // namespace celeritas
