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

// Shared oracle instantiated by three isolated CUDA translation units.

#include <app/app.h>
#include <collision_detection/aabb.h>
#include <cuda_tool/cuda_tool.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace test_bvh_scene_bounds
{
inline uint32_t expand_bits(uint32_t value)
{
    value = (value * 0x00010001u) & 0xFF0000FFu;
    value = (value * 0x00000101u) & 0x0F00F00Fu;
    value = (value * 0x00000011u) & 0xC30C30C3u;
    value = (value * 0x00000005u) & 0x49249249u;
    return value;
}

inline uint32_t morton3d(float x, float y, float z)
{
    x = std::fmin(std::fmax(x * 1024.0f, 0.0f), 1023.0f);
    y = std::fmin(std::fmax(y * 1024.0f, 0.0f), 1023.0f);
    z = std::fmin(std::fmax(z * 1024.0f, 0.0f), 1023.0f);
    return expand_bits(static_cast<uint32_t>(x)) * 4
           + expand_bits(static_cast<uint32_t>(y)) * 2
           + expand_bits(static_cast<uint32_t>(z));
}

inline float normalized_axis(float offset, float extent)
{
    return extent > 0.0f ? offset / extent : 0.0f;
}

inline std::vector<uint32_t> reference_morton_codes(const std::vector<AABB>& boxes,
                                                    const AABB& scene_box)
{
    std::vector<uint32_t> codes;
    codes.reserve(boxes.size());
    auto scene_min  = scene_box.min();
    auto scene_size = scene_box.sizes();
    for(const AABB& box : boxes)
    {
        auto center = box.center();
        codes.push_back(
            morton3d(normalized_axis(center.x() - scene_min.x(), scene_size.x()),
                     normalized_axis(center.y() - scene_min.y(), scene_size.y()),
                     normalized_axis(center.z() - scene_min.z(), scene_size.z())));
    }
    return codes;
}

inline std::vector<AABB> make_boxes(size_t count)
{
    std::vector<AABB> boxes(count);
    for(size_t i = 0; i < count; ++i)
    {
        float x = static_cast<float>(i % 19) * 0.25f - 2.0f;
        float y = static_cast<float>((i / 19) % 17) * 0.5f - 3.0f;
        float z = static_cast<float>((i * 7) % 23) * 0.125f - 1.0f;
        boxes[i].extend(Eigen::Vector3f{x, y, z});
        boxes[i].extend(Eigen::Vector3f{x + 0.125f, y + 0.25f, z + 0.5f});
    }

    if(!boxes.empty())
    {
        // Put the two global extrema at different indices (and, for large
        // inputs, different CTAs) so a reset/update race cannot hide.
        size_t min_index = count > 256 ? 17 : 0;
        size_t max_index = count - 1;
        boxes[min_index].extend(Eigen::Vector3f{-1024.0f, -2048.0f, -4096.0f});
        boxes[max_index].extend(Eigen::Vector3f{8192.0f, 4096.0f, 2048.0f});
    }
    return boxes;
}

inline AABB reference_scene_box(const std::vector<AABB>& boxes)
{
    AABB result;
    for(const AABB& box : boxes)
        result.extend(box);
    return result;
}

inline void check_scene_box(const AABB& actual, const AABB& expected)
{
    CHECK(actual.isEmpty() == expected.isEmpty());
    if(expected.isEmpty())
        return;

    for(int axis = 0; axis < 3; ++axis)
    {
        CHECK(actual.min()[axis] == expected.min()[axis]);
        CHECK(actual.max()[axis] == expected.max()[axis]);
    }
}

template <typename Impl>
void check_bounds_and_morton(const std::vector<AABB>& boxes)
{
    DeviceBuffer<AABB> device_boxes(boxes.size());
    if(!boxes.empty())
        device_boxes.view().copy_from(boxes.data());

    DeviceVar<AABB> scene_box;
    Impl::calcMaxBVFromBox(device_boxes.view(), scene_box.view());
    AABB actual_scene   = scene_box;
    AABB expected_scene = reference_scene_box(boxes);
    check_scene_box(actual_scene, expected_scene);

    DeviceBuffer<uint32_t> device_codes(boxes.size());
    Impl::calcMCsFromBox(device_boxes.view(), scene_box.cview(), device_codes.view());
    if(!boxes.empty())
    {
        std::vector<uint32_t> actual_codes(boxes.size());
        device_codes.view().copy_to(actual_codes.data());
        CHECK(actual_codes == reference_morton_codes(boxes, expected_scene));
    }
}

template <typename Impl>
void check_count_matrix()
{
    constexpr std::array<size_t, 15> counts = {
        0, 1, 17, 31, 32, 33, 64, 96, 127, 128, 255, 256, 257, 512, 513};
    for(size_t count : counts)
        check_bounds_and_morton<Impl>(make_boxes(count));
}

template <typename Impl>
void check_degenerate_axes()
{
    std::vector<AABB> points(33);
    for(AABB& box : points)
        box.extend(Eigen::Vector3f{3.0f, -2.0f, 7.0f});
    check_bounds_and_morton<Impl>(points);

    std::vector<AABB> linear(65);
    for(size_t i = 0; i < linear.size(); ++i)
        linear[i].extend(Eigen::Vector3f{static_cast<float>(i), 4.0f, -8.0f});
    check_bounds_and_morton<Impl>(linear);

    std::vector<AABB> planar(257);
    for(size_t i = 0; i < planar.size(); ++i)
    {
        float x = static_cast<float>(i % 17);
        float y = static_cast<float>(i / 17);
        planar[i].extend(Eigen::Vector3f{x, y, 2.0f});
    }
    check_bounds_and_morton<Impl>(planar);
}

template <typename Impl>
void check_signed_zero()
{
    // Numeric comparison classifies -0.0f as non-negative, while the float
    // atomic ordering must follow its sign bit. The old atomic max left the
    // scene maximum at a negative value for this finite, multi-CTA input.
    const float       negative_zero = std::copysign(0.0f, -1.0f);
    std::vector<AABB> boxes(257);
    for(size_t i = 0; i < boxes.size(); ++i)
    {
        const float lower = -static_cast<float>(i + 1);
        boxes[i].extend(Eigen::Vector3f{lower, lower - 1.0f, lower - 2.0f});
        boxes[i].extend(Eigen::Vector3f{negative_zero, negative_zero, negative_zero});
    }
    check_bounds_and_morton<Impl>(boxes);
}

template <typename Impl>
void check_reused_output()
{
    DeviceVar<AABB> scene_box;
    for(size_t count : {size_t{513}, size_t{0}, size_t{1}, size_t{255}})
    {
        auto               boxes = make_boxes(count);
        DeviceBuffer<AABB> device_boxes(boxes.size());
        if(!boxes.empty())
            device_boxes.view().copy_from(boxes.data());

        Impl::calcMaxBVFromBox(device_boxes.view(), scene_box.view());
        AABB actual_scene = scene_box;
        check_scene_box(actual_scene, reference_scene_box(boxes));
    }
}

template <typename Impl>
void check_impl()
{
    check_count_matrix<Impl>();
    check_degenerate_axes<Impl>();
    check_signed_zero<Impl>();
    check_reused_output<Impl>();
}
}  // namespace test_bvh_scene_bounds
