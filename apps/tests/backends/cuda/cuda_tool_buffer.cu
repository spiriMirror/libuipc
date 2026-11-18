/*
 * NVIDIA contributions: the DeviceReduce::Sum regression test for an empty
 * reused reduction range.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

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

#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/cub.h>

#include <app/app.h>

#include <array>

namespace
{
using uipc::backend::cuda_tool::DeviceBuffer;
using uipc::backend::cuda_tool::DeviceReduce;
using uipc::backend::cuda_tool::DeviceVar;

TEST_CASE("cuda_tool DeviceVector growth policies", "[cuda][cuda_tool][buffer]")
{
    SECTION("resize keeps value initialization")
    {
        DeviceBuffer<int> values;
        values.resize(4, 7);
        values.resize(2);
        values.resize(4);

        std::vector<int> host;
        values.copy_to(host);
        REQUIRE(host == std::vector<int>{7, 7, 0, 0});
    }

    SECTION("resize with an explicit value initializes only once logically")
    {
        DeviceBuffer<int> values;
        values.resize(5, 9);

        std::vector<int> host;
        values.copy_to(host);
        REQUIRE(host == std::vector<int>(5, 9));
    }

    SECTION("discard resize grows from the latest requirement")
    {
        DeviceBuffer<int> values;
        values.resize_discard(10);

        REQUIRE(values.size() == 10);
        REQUIRE(values.capacity() == 15);

        auto* pointer = values.data();
        values.resize_discard(3);
        values.resize_discard(12);
        REQUIRE(values.data() == pointer);
        REQUIRE(values.capacity() == 15);

        values.resize_discard(16);
        REQUIRE(values.size() == 16);
        REQUIRE(values.capacity() == 24);
    }

    SECTION("preserve resize retains the previous logical range")
    {
        DeviceBuffer<int> values;
        values.resize(4, 3);
        values.resize_preserve(10);

        REQUIRE(values.size() == 10);
        REQUIRE(values.capacity() == 15);

        std::vector<int> host(4);
        values.cview(0, 4).copy_to(host.data());
        REQUIRE(host == std::vector<int>(4, 3));
    }

    SECTION("amortized reserve does not change the logical size")
    {
        DeviceBuffer<int> values;
        values.resize(4, 5);
        values.reserve_amortized(10);

        REQUIRE(values.size() == 4);
        REQUIRE(values.capacity() == 15);

        std::vector<int> host;
        values.copy_to(host);
        REQUIRE(host == std::vector<int>(4, 5));
    }

    SECTION("discard reserve keeps the logical size and requested capacity")
    {
        DeviceBuffer<int> values;
        values.resize(4, 5);
        values.reserve_discard(11);

        REQUIRE(values.size() == 4);
        REQUIRE(values.capacity() == 11);

        values.resize_discard(11);
        REQUIRE(values.capacity() == 11);
    }
}

TEST_CASE("cuda_tool DeviceReduce Sum writes zero for an empty reused range", "[cuda][cuda_tool][reduce]")
{
    DeviceBuffer<double> values;
    DeviceVar<double>    sum{41.0};
    DeviceReduce         reduce;

    const std::array<double, 2> first = {1.25, 2.75};
    values.copy_from(first.data(), first.size());
    reduce.Sum(values.data(), sum.data(), values.size());
    REQUIRE(static_cast<double>(sum) == 4.0);

    values.resize_discard(0);
    reduce.Sum(values.data(), sum.data(), values.size());
    REQUIRE(static_cast<double>(sum) == 0.0);

    const std::array<double, 2> second = {8.0, -3.5};
    values.copy_from(second.data(), second.size());
    reduce.Sum(values.data(), sum.data(), values.size());
    REQUIRE(static_cast<double>(sum) == 4.5);
}
}  // namespace
