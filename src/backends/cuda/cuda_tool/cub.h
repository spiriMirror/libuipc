/*
 * NVIDIA contributions: zero-output handling for empty DeviceReduce::Sum,
 * optional radix-sort bit ranges and single checked cub calls on the cached
 * workspace, with the size query only when it is too small.
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

#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/buffer.h>
#include <cuda_runtime.h>
#include <cstddef>
#include <unordered_map>
// cub is a header-only CUDA algorithm library that ships with the CUDA toolkit.
#include <cub/device/device_reduce.cuh>
#include <cub/device/device_scan.cuh>
#include <cub/device/device_select.cuh>
#include <cub/device/device_partition.cuh>
#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_merge_sort.cuh>
#include <cub/device/device_run_length_encode.cuh>
// warp-level primitives used directly by backend kernels (spmv etc.)
#include <cub/warp/warp_reduce.cuh>
#include <cub/warp/warp_scan.cuh>
// NOTE: cub::DeviceSpmv was removed in CUDA 13's CCCL. The backend never actually
// calls DeviceSpmv (LinearSystemContext::spmv is unused), so it is intentionally
// not wrapped here. Use a custom SpMV kernel if one is ever needed.

namespace uipc::backend::cuda_tool
{
namespace details
{
    // Per-stream cached scratch for cub temp storage (muda Stream::workspace
    // parity): grows on demand and is reused across calls. Plain
    // cudaMalloc/cudaFree per call not only costs ~10-100us each, it also
    // serializes with the device -- measurable per frame with dozens of cub
    // calls (trajectory filters, matrix conversion).
    inline DeviceVector<std::byte>& cub_workspace(cudaStream_t stream)
    {
        static thread_local std::unordered_map<cudaStream_t, DeviceVector<std::byte>> workspaces;
        return workspaces[stream];
    }

    inline std::byte* cub_temp_storage(size_t bytes, cudaStream_t stream)
    {
        auto& buf = cub_workspace(stream);
        if(buf.capacity() < bytes)
            buf.reserve_discard(bytes * 2, stream);
        return reinterpret_cast<std::byte*>(buf.data());
    }

    // Run on the stream's workspace in one call: cub sizes its temporaries
    // on every call and, when the storage it is given is too small, returns
    // cudaErrorInvalidValue before it launches anything. Only then query,
    // grow the workspace and run. Where the temporaries live changes no
    // result; the size query would repeat the dispatch's host work.
    template <typename Fn>
    void run_with_temp_storage(Fn&& fn, cudaStream_t stream)
    {
        auto& buf = cub_workspace(stream);
        if(buf.capacity() > 0)
        {
            size_t capacity = buf.capacity();
            cudaError_t e = fn(reinterpret_cast<std::byte*>(buf.data()), capacity, stream);
            if(e == cudaSuccess)
                return;
            if(e != cudaErrorInvalidValue)
                CUDA_TOOL_CHECK(e);
        }
        size_t temp_bytes = 0;
        CUDA_TOOL_CHECK(fn(nullptr, temp_bytes, stream));  // query
        std::byte* temp = temp_bytes ? cub_temp_storage(temp_bytes, stream) : nullptr;
        CUDA_TOOL_CHECK(fn(temp, temp_bytes, stream));  // run
    }
}  // namespace details

// Thin raw-CUDA wrappers over cub. Each returns a small builder bound to a stream,
// mirroring the cuda_tool::Device* API shape used across the backend.

class DeviceReduce
{
  public:
    explicit DeviceReduce(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }

    template <typename T, typename Op>
    DeviceReduce& Reduce(const T* in, T* out, int n, Op op, T init)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceReduce::Reduce(t, b, in, out, n, op, init, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& Sum(const T* in, T* out, int n)
    {
        // CUB does not promise to write the output for an empty reduction.
        // Callers reuse DeviceVar outputs across Line Search trials, so leaving
        // an old value here would turn an empty energy range into stale energy.
        if(n == 0)
        {
            CUDA_TOOL_CHECK(cudaMemsetAsync(out, 0, sizeof(T), m_stream));
            return *this;
        }
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceReduce::Sum(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& Min(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceReduce::Min(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& Max(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceReduce::Max(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& ArgMin(const T* in, cub::KeyValuePair<int, T>* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceReduce::ArgMin(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& ArgMax(const T* in, cub::KeyValuePair<int, T>* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceReduce::ArgMax(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

class DeviceScan
{
  public:
    explicit DeviceScan(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }
    template <typename T>
    DeviceScan& InclusiveSum(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceScan::InclusiveSum(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceScan& ExclusiveSum(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceScan::ExclusiveSum(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T, typename Op>
    DeviceScan& InclusiveScan(const T* in, T* out, Op op, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceScan::InclusiveScan(t, b, in, out, op, n, s); },
            m_stream);
        return *this;
    }
    template <typename T, typename Op>
    DeviceScan& ExclusiveScan(const T* in, T* out, Op op, T init, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                return cub::DeviceScan::ExclusiveScan(t, b, in, out, op, init, n, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

class DeviceSelect
{
  public:
    explicit DeviceSelect(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }
    template <typename T, typename Pred>
    DeviceSelect& If(const T* in, T* out, int* num_selected, int n, Pred pred)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                return cub::DeviceSelect::If(t, b, in, out, num_selected, n, pred, s);
            },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceSelect& Flagged(const T* in, const int* flags, T* out, int* num_selected, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                return cub::DeviceSelect::Flagged(t, b, in, flags, out, num_selected, n, s);
            },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceSelect& Unique(const T* in, T* out, int* num_selected, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                return cub::DeviceSelect::Unique(t, b, in, out, num_selected, n, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

class DevicePartition
{
  public:
    explicit DevicePartition(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }
    template <typename T, typename Pred>
    DevicePartition& If(const T* in, T* out, int* num_selected, int n, Pred pred)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                return cub::DevicePartition::If(t, b, in, out, num_selected, n, pred, s);
            },
            m_stream);
        return *this;
    }
    template <typename T>
    DevicePartition& Flagged(const T* in, const int* flags, T* out, int* num_selected, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                return cub::DevicePartition::Flagged(t, b, in, flags, out, num_selected, n, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

class DeviceRadixSort
{
  public:
    explicit DeviceRadixSort(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }
    template <typename K>
    DeviceRadixSort& SortKeys(const K* in, K* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                return cub::DeviceRadixSort::SortKeys(t, b, in, out, n, 0, sizeof(K) * 8, s);
            },
            m_stream);
        return *this;
    }
    template <typename K, typename V>
    DeviceRadixSort& SortPairs(const K* kin,
                               K*       kout,
                               const V* vin,
                               V*       vout,
                               int      n,
                               int      begin_bit = 0,
                               int      end_bit   = sizeof(K) * 8)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                return cub::DeviceRadixSort::SortPairs(
                    t, b, kin, kout, vin, vout, n, begin_bit, end_bit, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

class DeviceMergeSort
{
  public:
    explicit DeviceMergeSort(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }
    template <typename K, typename Cmp>
    DeviceMergeSort& SortKeys(K* keys, int n, Cmp cmp)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { return cub::DeviceMergeSort::SortKeys(t, b, keys, n, cmp, s); },
            m_stream);
        return *this;
    }
    template <typename K, typename V, typename Cmp>
    DeviceMergeSort& SortPairs(K* keys, V* vals, int n, Cmp cmp)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                return cub::DeviceMergeSort::SortPairs(t, b, keys, vals, n, cmp, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

class DeviceRunLengthEncode
{
  public:
    explicit DeviceRunLengthEncode(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }
    template <typename T, typename CountT>
    DeviceRunLengthEncode& Encode(const T* in, T* unique, CountT* counts, int* num_runs, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                return cub::DeviceRunLengthEncode::Encode(
                    t, b, in, unique, counts, num_runs, n, s);
            },
            m_stream);
        return *this;
    }
    template <typename T, typename OffT, typename LenT>
    DeviceRunLengthEncode& NonTrivialRuns(const T* in, OffT* offsets, LenT* lengths, int* num_runs, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                return cub::DeviceRunLengthEncode::NonTrivialRuns(
                    t, b, in, offsets, lengths, num_runs, n, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

}  // namespace uipc::backend::cuda_tool
