//------------------------------- -*- C++ -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/gen/detail/ScratchExecute.device.hh
//! \brief Thrust execution policy backed by a persistent scratch arena.
//---------------------------------------------------------------------------//
#pragma once

#include <algorithm>
#include <cstddef>
#include <thrust/execution_policy.h>
#include <thrust/mr/allocator.h>
#include <thrust/mr/memory_resource.h>
#include <thrust/version.h>

#include "corecel/DeviceRuntimeApi.hh"

#include "corecel/Macros.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/Stream.hh"
#include "corecel/sys/detail/AsyncMemoryResource.hh"

#include "GeneratorScratch.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Bump allocator over a \c GeneratorScratch arena.
 *
 * Construction grows the arena to the last recorded demand (safe: no
 * suballocation from an earlier call is live, and the DeviceVector's
 * stream-ordered free/alloc sequence after the prior call's kernels on
 * the same stream) and resets the cursor. Allocation is a cursor bump;
 * deallocation inside the arena is a no-op (temporaries die with the
 * algorithm call). Requests past the arena's end fall back to the
 * stream's async pool and raise the recorded demand, so the next call
 * grows past it. One instance serves one algorithm call.
 */
class ScratchMr final : public thrust::mr::memory_resource<void*>
{
  public:
    ScratchMr(GeneratorScratch& scratch, StreamId stream)
        : scratch_{scratch}, stream_{stream}
    {
        if (scratch_.need > scratch_.temp.size())
        {
            scratch_.temp = DeviceVector<char>(
                std::max(scratch_.need,
                         std::max<std::size_t>(2 * scratch_.temp.size(), 4096)),
                stream_);
        }
        scratch_.need = 0;
    }

    void* do_allocate(std::size_t bytes, std::size_t align) final
    {
        std::size_t start = (offset_ + align - 1) / align * align;
        scratch_.need = std::max(scratch_.need, start + bytes);
        if (start + bytes <= scratch_.temp.size())
        {
            offset_ = start + bytes;
            return scratch_.temp.data() + start;
        }
        return celeritas::device()
            .stream(stream_)
            .memory_resource()
            .do_allocate(bytes, align);
    }

    void do_deallocate(void* p, std::size_t bytes, std::size_t align) final
    {
        auto* c = static_cast<char*>(p);
        if (c >= scratch_.temp.data()
            && c < scratch_.temp.data() + scratch_.temp.size())
        {
            return;
        }
        celeritas::device().stream(stream_).memory_resource().do_deallocate(
            p, bytes, align);
    }

  private:
    GeneratorScratch& scratch_;
    StreamId stream_;
    std::size_t offset_{0};
};

//---------------------------------------------------------------------------//
/*!
 * Execute thrust on the stream with temporaries from the scratch arena.
 *
 * Mirrors \c thrust_execute_on, swapping the stream's pool allocator for
 * the persistent arena. The \c ScratchMr must outlive the algorithm call.
 */
inline auto thrust_execute_scratch(ScratchMr* mr, StreamId stream_id)
{
    using Alloc = thrust::mr::allocator<char, ScratchMr>;
#if THRUST_VERSION >= 101600
    // Newer thrust supports asynchronous par
    auto& par_nosync = thrust::CELER_DEVICE_PLATFORM::par_nosync;
#else
    // Fall back to synchronous execution
    auto& par_nosync = thrust::CELER_DEVICE_PLATFORM::par;
#endif
    return par_nosync(Alloc(mr)).on(
        celeritas::device().stream(stream_id).get());
}

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
