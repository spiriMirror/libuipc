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

#include <uipc/common/log.h>
#include <uipc/common/type_define.h>

#include <cstdint>
#include <limits>
#include <type_traits>

namespace uipc::backend::cuda
{
template <typename Index>
struct ContactTypeBlockLayout
{
    Index pt_end;
    Index ee_offset;
    Index ee_end;
    Index pe_offset;
    Index pe_end;
    Index pp_offset;
    Index padded_total;
};

template <typename Index>
struct ContactTypeContiguousLayout
{
    Index pt_end;
    Index ee_end;
    Index pe_end;
    Index pp_end;
};

// Build the prefix ends used by a fused PT/EE/PE/PP kernel without narrowing
// any individual count or overflowing the signed device index type.
template <typename Index>
inline ContactTypeContiguousLayout<Index> make_contact_type_contiguous_layout(
    SizeT pt_count, SizeT ee_count, SizeT pe_count, SizeT pp_count)
{
    static_assert(std::is_integral_v<Index> && std::is_signed_v<Index>);

    constexpr SizeT IndexMax = static_cast<SizeT>(std::numeric_limits<Index>::max());
    SizeT total  = 0;
    auto  append = [&](SizeT count)
    {
        UIPC_ASSERT_THROW(count <= IndexMax - total,
                          "Fused contact count exceeds index limit {}: "
                          "prefix={}, next={}",
                          IndexMax,
                          total,
                          count);
        total += count;
        return static_cast<Index>(total);
    };

    ContactTypeContiguousLayout<Index> layout{};
    layout.pt_end = append(pt_count);
    layout.ee_end = append(ee_count);
    layout.pe_end = append(pe_count);
    layout.pp_end = append(pp_count);
    return layout;
}

template <int BlockSize, typename Index>
inline ContactTypeBlockLayout<Index> make_contact_type_block_layout(
    Index pt_count, Index ee_count, Index pe_count, Index pp_count)
{
    static_assert(BlockSize > 0);
    static_assert(std::is_integral_v<Index> && std::is_signed_v<Index>);
    static_assert(sizeof(Index) < sizeof(std::int64_t));

    UIPC_ASSERT_THROW(pt_count >= 0 && ee_count >= 0 && pe_count >= 0 && pp_count >= 0,
                      "Contact counts must be nonnegative: PT={}, EE={}, PE={}, PP={}",
                      pt_count,
                      ee_count,
                      pe_count,
                      pp_count);

    using Wide          = std::int64_t;
    const auto round_up = [](Wide count)
    {
        return ((count + Wide(BlockSize) - Wide(1)) / Wide(BlockSize)) * Wide(BlockSize);
    };

    const Wide pt_count_wide = static_cast<Wide>(pt_count);
    const Wide ee_count_wide = static_cast<Wide>(ee_count);
    const Wide pe_count_wide = static_cast<Wide>(pe_count);
    const Wide pp_count_wide = static_cast<Wide>(pp_count);

    ContactTypeBlockLayout<Wide> wide_layout{};
    wide_layout.pt_end       = pt_count_wide;
    wide_layout.ee_offset    = round_up(pt_count_wide);
    wide_layout.ee_end       = wide_layout.ee_offset + ee_count_wide;
    wide_layout.pe_offset    = wide_layout.ee_offset + round_up(ee_count_wide);
    wide_layout.pe_end       = wide_layout.pe_offset + pe_count_wide;
    wide_layout.pp_offset    = wide_layout.pe_offset + round_up(pe_count_wide);
    wide_layout.padded_total = wide_layout.pp_offset + pp_count_wide;

    constexpr Wide IndexMax = static_cast<Wide>(std::numeric_limits<Index>::max());
    UIPC_ASSERT_THROW(wide_layout.padded_total <= IndexMax,
                      "Padded contact count {} exceeds index limit {}",
                      wide_layout.padded_total,
                      IndexMax);

    ContactTypeBlockLayout<Index> layout{};
    layout.pt_end       = static_cast<Index>(wide_layout.pt_end);
    layout.ee_offset    = static_cast<Index>(wide_layout.ee_offset);
    layout.ee_end       = static_cast<Index>(wide_layout.ee_end);
    layout.pe_offset    = static_cast<Index>(wide_layout.pe_offset);
    layout.pe_end       = static_cast<Index>(wide_layout.pe_end);
    layout.pp_offset    = static_cast<Index>(wide_layout.pp_offset);
    layout.padded_total = static_cast<Index>(wide_layout.padded_total);
    return layout;
}
}  // namespace uipc::backend::cuda
