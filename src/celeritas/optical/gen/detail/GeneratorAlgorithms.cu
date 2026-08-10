//------------------------------ -*- cuda -*- -------------------------------//
// Copyright Celeritas contributors: see top-level COPYRIGHT file for details
// SPDX-License-Identifier: (Apache-2.0 OR MIT)
//---------------------------------------------------------------------------//
//! \file celeritas/optical/gen/detail/GeneratorAlgorithms.cu
//---------------------------------------------------------------------------//
#include "GeneratorAlgorithms.hh"

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/transform_scan.h>

#include "corecel/Assert.hh"
#include "corecel/Macros.hh"
#include "corecel/data/Copier.hh"
#include "corecel/math/Algorithms.hh"
#include "corecel/sys/Device.hh"
#include "corecel/sys/ScopedProfiling.hh"
#include "corecel/sys/Thrust.device.hh"
#include "celeritas/optical/WavelengthShiftData.hh"

#include "ScratchExecute.device.hh"

#include "../GeneratorData.hh"

namespace celeritas
{
namespace optical
{
namespace detail
{
//---------------------------------------------------------------------------//
/*!
 * Calculate the inclusive prefix sum of the number of optical photons.
 *
 * \return Total accumulated value
 */
template<class T>
size_type
inclusive_scan_photons(ItemsRef<T, MemSpace::device> const& buffer,
                       ItemsRef<size_type, MemSpace::device> const& offsets,
                       size_type size,
                       GeneratorScratch* scratch,
                       StreamId stream)
{
    CELER_EXPECT(!buffer.empty());
    CELER_EXPECT(size > 0 && size <= buffer.size());
    CELER_EXPECT(offsets.size() == buffer.size());

    ScopedProfiling profile_this{"prefix-sum-counts"};
    auto data = thrust::device_pointer_cast(buffer.data().get());
    auto result = thrust::device_pointer_cast(offsets.data().get());
    auto scan = [&](auto&& policy) {
        thrust::transform_inclusive_scan(policy,
                                         data,
                                         data + size,
                                         result,
                                         GetNumPhotons<T>{},
                                         thrust::plus<size_type>());
    };
    if (scratch)
    {
        // Temporaries come from the persistent arena instead of cycling
        // through the stream pool every call
        ScratchMr mr{*scratch, stream};
        scan(thrust_execute_scratch(&mr, stream));
    }
    else
    {
        scan(thrust_execute_on(stream));
    }
    CELER_DEVICE_API_CALL(PeekAtLastError());

    // Copy the last element (accumulated total) back to host
    return ItemCopier<size_type>{stream}((result + size - 1).get());
}

//---------------------------------------------------------------------------//
// EXPLICIT INSTANTIATION
//---------------------------------------------------------------------------//

template size_type inclusive_scan_photons(
    ItemsRef<GeneratorDistributionData, MemSpace::device> const&,
    ItemsRef<size_type, MemSpace::device> const&,
    size_type,
    GeneratorScratch*,
    StreamId);
template size_type
inclusive_scan_photons(ItemsRef<WlsDistributionData, MemSpace::device> const&,
                       ItemsRef<size_type, MemSpace::device> const&,
                       size_type,
                       GeneratorScratch*,
                       StreamId);

//---------------------------------------------------------------------------//
}  // namespace detail
}  // namespace optical
}  // namespace celeritas
