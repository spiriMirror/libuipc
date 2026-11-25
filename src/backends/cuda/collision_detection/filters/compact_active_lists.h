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
#include <uipc/backend/macro.h>
#include <type_define.h>
#include <cuda_runtime.h>
#include <array>

namespace uipc::backend::cuda
{
// One list of filter_active's output: n elements of `ints` indices each, of
// which an element is kept unless its first index is -1.
struct CompactActiveList
{
    const IndexT* in;
    IndexT*       out;    // the kept elements, in input order
    IndexT*       count;  // device: number of kept elements
    int           n;
    int           ints;
    IndexT*       count_copy = nullptr;  // optional: the count stored here too
};

// Longest list compact_active_lists() takes.
constexpr int CompactActiveListMaxSize = 16384;

// Compacts four lists in one launch on `stream`, with the outputs and counts
// of DeviceSelect::If (stable) on each. Returns false, launching nothing, if a
// list is longer than CompactActiveListMaxSize.
UIPC_BACKEND_API bool compact_active_lists(const std::array<CompactActiveList, 4>& lists,
                                           cudaStream_t stream);
}  // namespace uipc::backend::cuda
