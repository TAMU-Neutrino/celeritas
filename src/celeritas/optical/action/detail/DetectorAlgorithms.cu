//------------------------------ -*- cuda -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/action/detail/DetectorAlgorithms.cu
//---------------------------------------------------------------------------//
#include "DetectorAlgorithms.hh"

#if CELERITAS_USE_CUDA
#    include <cub/device/device_select.cuh>
#elif CELERITAS_HAVE_HIPCUB
#    include <hipcub/device/device_select.hpp>
#else
#    include <thrust/copy.h>
#    include <thrust/execution_policy.h>
#endif
#include <thrust/device_ptr.h>

#include "corecel/Macros.hh"
#include "corecel/data/Copier.hh"
#include "corecel/data/DeviceVector.hh"
#include "corecel/data/ObserverPtr.device.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/ScopedProfiling.hh"
#include "corecel/sys/Stream.hh"
#include "corecel/sys/Thrust.device.hh"

#if CELERITAS_HAVE_HIPCUB
namespace cub = hipcub;
#endif

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Stably compact the valid hits into the buffer's device storage.
 *
 * The selection is stable in both implementations, so the compacted order
 * is the ascending slot order the host-side filter used to produce: the
 * delivered hits are element-for-element what they always were. The
 * caller already knows how many hits to expect -- every valid hit bumped
 * the cumulative counter in the same kernel pass that wrote it -- so the
 * selection's own count is only cross-checked, and only where checking is
 * free or debug is on.
 */
void copy_if_hit(DetectorStateRef<MemSpace::device> const& detectors,
                 DetectorHitBuffer* buf,
                 size_type num_expected,
                 StreamId stream_id)
{
    CELER_EXPECT(buf);
    CELER_EXPECT(buf->compact.size() == detectors.detector_hits.size());
    CELER_EXPECT(num_expected <= detectors.detector_hits.size());

    ScopedProfiling profile_this{"copy-if-hit"};
    auto hits = device_pointer_cast(detectors.detector_hits.data());
    auto compacted = thrust::device_pointer_cast(buf->compact.data());
    size_type const num_slots = detectors.detector_hits.size();
#ifdef CELER_USE_THRUST
    auto end = thrust::copy_if(thrust_execute_on(stream_id),
                               hits,
                               hits + num_slots,
                               compacted,
                               IsHit{});
    CELER_DEVICE_API_CALL(PeekAtLastError());
    CELER_ASSERT(static_cast<size_type>(end - compacted) == num_expected);
#else
    CELER_EXPECT(!buf->result.empty());
    auto& stream = device().stream(stream_id);
    // Calling with nullptr causes the function to return the amount of
    // working space needed instead of invoking the kernel; the size is a
    // function of the fixed slot capacity, so the storage grown here on
    // the first call serves every later one.
    size_t temp_storage_bytes = 0;
    cub::DeviceSelect::If(nullptr,
                          temp_storage_bytes,
                          hits,
                          compacted,
                          buf->result.data(),
                          num_slots,
                          IsHit{},
                          stream.get());
    if (buf->temp.size() < temp_storage_bytes)
    {
        buf->temp = DeviceVector<char>(temp_storage_bytes, stream_id);
    }
    cub::DeviceSelect::If(buf->temp.data(),
                          temp_storage_bytes,
                          hits,
                          compacted,
                          buf->result.data(),
                          num_slots,
                          IsHit{},
                          stream.get());
    CELER_DEVICE_API_CALL(PeekAtLastError());
#    if CELERITAS_DEBUG
    auto selected = ItemCopier<size_type>{stream_id}(buf->result.data());
    stream.sync();
    CELER_ASSERT(selected == num_expected);
#    else
    CELER_DISCARD(num_expected);
#    endif
#endif
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
