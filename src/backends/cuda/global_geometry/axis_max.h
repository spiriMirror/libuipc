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
#include <type_define.h>

namespace uipc::backend::cuda
{
// Device side of GlobalVertexManager::VertexDisplacementInfo::axis_max():
// folds |d| per axis of one thread's displacement into the slot (the bits of
// a non-negative double order as its value, so atomicMax on them takes the
// maximum exactly, whatever the order). Every thread of the warp must call it
// (an inactive thread passes `active` false); `clear`, when not null, is
// zeroed by thread 0 of block 0 (the next collection's slot).
__device__ inline void axis_max_add(unsigned long long* slot,
                                    unsigned long long* clear,
                                    bool                active,
                                    const Vector3&      d)
{
    double m = 0.0;
    if(active)
    {
        // the reduction's operator: the larger absolute value
        double a = fabs(d(0)), b = fabs(d(1)), c = fabs(d(2));
        m = a > b ? a : b;
        m = m > c ? m : c;
    }
    unsigned long long bits = __double_as_longlong(m);
    for(int offset = 16; offset > 0; offset >>= 1)
    {
        unsigned long long other = __shfl_down_sync(0xffffffffu, bits, offset);
        bits                     = other > bits ? other : bits;
    }
    if((threadIdx.x & 31) == 0 && bits != 0)
        atomicMax(slot, bits);
    if(clear && blockIdx.x == 0 && threadIdx.x == 0)
        *clear = 0;
}
}  // namespace uipc::backend::cuda
