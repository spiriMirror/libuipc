/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <cfloat>
#include <type_define.h>
#include <cuda_runtime.h>

namespace uipc::backend::cuda
{
// The minimum of a filter's TOI values, taken by the TOI kernels themselves
// instead of a DeviceReduce::Min launch after them: every warp folds its
// lanes' values and its lane 0 takes atomicMin of an order-preserving 64-bit
// key into the filter's slot; the filter's last TOI launch (the finishing
// one) also counts its blocks, and its last block writes the minimum to the
// output and resets the slot. Launches of one filter run in stream order, so
// the finishing launch sees the earlier launches' keys. With `blocks` set, the
// launches may run concurrently: each passes the output, every block counts,
// and the last of those `blocks` blocks finishes. The minimum of values
// without NaN is unique up to the sign of zero, so it equals
// DeviceReduce::Min over the same values.
struct TOIMinSlot
{
    unsigned long long* key;  // the running minimum's key, TOIMinIdentity between uses
    unsigned long long* done;  // blocks of the finishing launch that are done, 0 between uses
    // blocks of all launches that finish together (0: the finishing launch's own)
    unsigned long long blocks = 0;
};

// Larger than the key of any value (also +inf); the two words of a slot start
// as {TOIMinIdentity, 0}.
constexpr unsigned long long TOIMinIdentity = ~0ull;

// Unsigned order of the keys = numeric order of the doubles (-0 before +0).
__device__ inline unsigned long long toi_min_key(double x)
{
    auto b = static_cast<unsigned long long>(__double_as_longlong(x));
    return (b >> 63) ? ~b : (b | 0x8000000000000000ull);
}

__device__ inline double toi_min_value(unsigned long long k)
{
    auto b = (k >> 63) ? (k & 0x7FFFFFFFFFFFFFFFull) : ~k;
    return __longlong_as_double(static_cast<long long>(b));
}

// Called once by every thread of every block (blockDim.x a multiple of 32);
// threads without a value pass TOIMinIdentity. `out` is the output of the
// finishing launch and nullptr in the others.
__device__ inline void toi_min_block(unsigned long long key, TOIMinSlot slot, double* out)
{
    for(int o = 16; o > 0; o >>= 1)
    {
        unsigned long long other = __shfl_xor_sync(0xffffffffu, key, o);
        key                      = other < key ? other : key;
    }
    if((threadIdx.x & 31) == 0 && key != TOIMinIdentity)
    {
        atomicMin(slot.key, key);
        __threadfence();
    }
    if(out == nullptr)
        return;
    __syncthreads();
    if(threadIdx.x == 0)
    {
        __threadfence();
        const unsigned long long blocks = slot.blocks ? slot.blocks : gridDim.x;
        if(atomicAdd(slot.done, 1ull) == blocks - 1)  // the last block
        {
            // no value at all: DBL_MAX, as DeviceReduce::Min for an empty input
            const unsigned long long k = atomicExch(slot.key, TOIMinIdentity);
            *out       = k == TOIMinIdentity ? DBL_MAX : toi_min_value(k);
            *slot.done = 0;
        }
    }
}
}  // namespace uipc::backend::cuda
