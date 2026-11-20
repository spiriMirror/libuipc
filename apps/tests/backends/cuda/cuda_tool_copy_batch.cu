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

// cuda_tool::CopyBatch (the copies that open a line search): every range of a
// batch ends with exactly the bytes of its source (or zeros), whether the
// kernel or a fallback copy moved it.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/copy_batch.h>

#include <cstring>
#include <stdexcept>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
struct Triple
{
    double x, y, z;
};

template <typename T>
std::vector<T> host_of(const cuda_tool::DeviceBuffer<T>& d)
{
    std::vector<T> h;
    d.copy_to(h);
    return h;
}

template <typename T>
bool same_bytes(const std::vector<T>& a, const std::vector<T>& b)
{
    return a.size() == b.size()
           && std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
}

TEST_CASE("cuda_tool CopyBatch copies and zero-fills ranges in one launch", "[cuda][cuda_tool][copy_batch]")
{
    std::vector<double> h_a(1000), h_garbage(1000, -7.5);
    for(size_t i = 0; i < h_a.size(); ++i)
        h_a[i] = 1.0 / (i + 1) + 1e-300 * i;  // subnormal-free, every bit pattern different
    std::vector<Triple> h_t(1237);
    for(size_t i = 0; i < h_t.size(); ++i)
        h_t[i] = Triple{double(i), -double(i), 0.5 * i};
    std::vector<int> h_i = {1, 2, 3};  // 12 bytes: not whole 8-byte words

    cuda_tool::DeviceBuffer<double> src_a, dst_a, zeroed;
    cuda_tool::DeviceBuffer<Triple> src_t, dst_t;
    cuda_tool::DeviceBuffer<int>    src_i, dst_i;
    src_a.copy_from(h_a.data(), h_a.size());
    dst_a.copy_from(h_garbage.data(), h_garbage.size());
    zeroed.copy_from(h_garbage.data(), h_garbage.size());
    src_t.copy_from(h_t.data(), h_t.size());
    dst_t.resize(h_t.size() + 10);
    src_i.copy_from(h_i.data(), h_i.size());
    dst_i.resize(h_i.size());

    cuda_tool::CopyBatch batch;
    REQUIRE(batch.empty());
    batch.copy<double>(dst_a.view(0, 600), src_a.cview(400, 600));  // offset source
    batch.copy<double>(dst_a.view(600, 400), src_a.cview(0, 400));
    batch.copy<Triple>(dst_t.view(10, h_t.size()), src_t.cview());  // offset destination
    batch.zero<double>(zeroed.view(1, 998));
    batch.copy<int>(dst_i.view(), src_i.cview());  // fallback copy
    REQUIRE_FALSE(batch.empty());
    batch.launch();
    REQUIRE(batch.empty());

    std::vector<double> want_a(h_a.begin() + 400, h_a.end());
    want_a.insert(want_a.end(), h_a.begin(), h_a.begin() + 400);
    REQUIRE(same_bytes(host_of(dst_a), want_a));

    auto got_t = host_of(dst_t);
    REQUIRE(same_bytes(std::vector<Triple>(got_t.begin() + 10, got_t.end()), h_t));

    std::vector<double> want_zero(1000, 0.0);
    want_zero.front() = want_zero.back() = -7.5;
    REQUIRE(same_bytes(host_of(zeroed), want_zero));

    REQUIRE(host_of(dst_i) == h_i);

    // an empty batch launches nothing
    batch.launch();
}

TEST_CASE("cuda_tool CopyBatch rejects mismatched and too many ranges", "[cuda][cuda_tool][copy_batch]")
{
    std::vector<double> h_b(32);
    for(int i = 0; i < 32; ++i)
        h_b[i] = 0.25 * i - 3.0;
    cuda_tool::DeviceBuffer<double> a, b;
    a.resize(32);
    b.copy_from(h_b.data(), h_b.size());
    cuda_tool::CopyBatch batch;
    REQUIRE_THROWS_AS(batch.copy<double>(a.view(0, 3), b.cview(0, 4)), std::runtime_error);
    for(int i = 0; i < 16; ++i)
        batch.copy<double>(a.view(i, 1), b.cview(16 + i, 1));
    REQUIRE_THROWS_AS(batch.copy<double>(a.view(16, 1), b.cview(0, 1)), std::length_error);
    batch.launch();
    auto got = host_of(a);
    REQUIRE(same_bytes(std::vector<double>(got.begin(), got.begin() + 16),
                       std::vector<double>(h_b.begin() + 16, h_b.end())));
}
}  // namespace
