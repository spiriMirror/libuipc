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
    inline std::byte* cub_temp_storage(size_t bytes, cudaStream_t stream)
    {
        static thread_local std::unordered_map<cudaStream_t, DeviceVector<std::byte>> workspaces;
        auto& buf = workspaces[stream];
        if(buf.capacity() < bytes)
            buf.reserve_discard(bytes * 2, stream);
        return reinterpret_cast<std::byte*>(buf.data());
    }

    // query temp storage size, serve it from the per-stream workspace, run.
    template <typename Fn>
    void run_with_temp_storage(Fn&& fn, cudaStream_t stream)
    {
        size_t temp_bytes = 0;
        fn(nullptr, temp_bytes, stream);  // query
        std::byte* temp = temp_bytes ? cub_temp_storage(temp_bytes, stream) : nullptr;
        fn(temp, temp_bytes, stream);  // run
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
            { cub::DeviceReduce::Reduce(t, b, in, out, n, op, init, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& Sum(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceReduce::Sum(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& Min(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceReduce::Min(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& Max(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceReduce::Max(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& ArgMin(const T* in, cub::KeyValuePair<int, T>* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceReduce::ArgMin(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceReduce& ArgMax(const T* in, cub::KeyValuePair<int, T>* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceReduce::ArgMax(t, b, in, out, n, s); },
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
            { cub::DeviceScan::InclusiveSum(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceScan& ExclusiveSum(const T* in, T* out, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceScan::ExclusiveSum(t, b, in, out, n, s); },
            m_stream);
        return *this;
    }
    template <typename T, typename Op>
    DeviceScan& InclusiveScan(const T* in, T* out, Op op, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceScan::InclusiveScan(t, b, in, out, op, n, s); },
            m_stream);
        return *this;
    }
    template <typename T, typename Op>
    DeviceScan& ExclusiveScan(const T* in, T* out, Op op, T init, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceScan::ExclusiveScan(t, b, in, out, op, init, n, s); },
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
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceSelect::If(t, b, in, out, num_selected, n, pred, s); },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceSelect& Flagged(const T* in, const int* flags, T* out, int* num_selected, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                cub::DeviceSelect::Flagged(t, b, in, flags, out, num_selected, n, s);
            },
            m_stream);
        return *this;
    }
    template <typename T>
    DeviceSelect& Unique(const T* in, T* out, int* num_selected, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceSelect::Unique(t, b, in, out, num_selected, n, s); },
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
                cub::DevicePartition::If(t, b, in, out, num_selected, n, pred, s);
            },
            m_stream);
        return *this;
    }
    template <typename T>
    DevicePartition& Flagged(const T* in, const int* flags, T* out, int* num_selected, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s) {
                cub::DevicePartition::Flagged(t, b, in, flags, out, num_selected, n, s);
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
            [&](void* t, size_t& b, cudaStream_t s) {
                cub::DeviceRadixSort::SortKeys(t, b, in, out, n, 0, sizeof(K) * 8, s);
            },
            m_stream);
        return *this;
    }
    template <typename K, typename V>
    DeviceRadixSort& SortPairs(const K* kin, K* kout, const V* vin, V* vout, int n)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            {
                cub::DeviceRadixSort::SortPairs(
                    t, b, kin, kout, vin, vout, n, 0, sizeof(K) * 8, s);
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
            { cub::DeviceMergeSort::SortKeys(t, b, keys, n, cmp, s); },
            m_stream);
        return *this;
    }
    template <typename K, typename V, typename Cmp>
    DeviceMergeSort& SortPairs(K* keys, V* vals, int n, Cmp cmp)
    {
        details::run_with_temp_storage(
            [&](void* t, size_t& b, cudaStream_t s)
            { cub::DeviceMergeSort::SortPairs(t, b, keys, vals, n, cmp, s); },
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
                cub::DeviceRunLengthEncode::Encode(t, b, in, unique, counts, num_runs, n, s);
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
                cub::DeviceRunLengthEncode::NonTrivialRuns(
                    t, b, in, offsets, lengths, num_runs, n, s);
            },
            m_stream);
        return *this;
    }

  private:
    cudaStream_t m_stream;
};

}  // namespace uipc::backend::cuda_tool
