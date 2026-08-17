#pragma once
// Minimal linear-system formats used by the backend's global linear solver.
// Raw CUDA containers + views; dot/norm reductions replace muda's cublas context.
#include <cuda_tool/buffer.h>
#include <cuda_tool/cub.h>
#include <cublas_v2.h>
#include <Eigen/Core>

namespace uipc::backend::cuda_tool
{
// A single sparse-matrix entry. Block matrices (M,N > 1) store an Eigen block as value.
template <typename T, int M, int N>
struct MatrixTriplet
{
    using ValueT = std::conditional_t<(M == 1 && N == 1), T, Eigen::Matrix<T, M, N>>;
    int    row_index;
    int    col_index;
    ValueT value;
};

template <typename T, int N>
struct VectorDoublet
{
    using ValueT = std::conditional_t<(N == 1), T, Eigen::Vector<T, N>>;
    int    index;
    ValueT value;
};

// ---------------------------------------------------------------------------
// Triplet matrix (COO, possibly block). Owns row/col indices + values.
// ---------------------------------------------------------------------------
template <typename T, int M = 1, int N = 1>
class DeviceTripletMatrix
{
  public:
    using ValueT   = std::conditional_t<(M == 1 && N == 1), T, Eigen::Matrix<T, M, N>>;
    using TripletT = MatrixTriplet<T, M, N>;

    void reshape(int rows, int cols)
    {
        m_rows = rows;
        m_cols = cols;
    }
    void resize_triplets(size_t n) { m_values.resize(n); }
    void reserve_triplets(size_t n)
    {
        m_values.reserve(n);
    }
    void clear()
    {
        m_rows = m_cols = 0;
        m_values.clear();
    }

    int    rows() const { return m_rows; }
    int    cols() const { return m_cols; }
    size_t triplet_count() const { return m_values.size(); }
    size_t triplet_capacity() const { return m_values.capacity(); }
    int    non_zeros() const { return (int)m_values.size(); }

    BufferView<ValueT>  values() { return m_values.view(); }
    CBufferView<ValueT> values() const { return m_values.cview(); }
    BufferView<int>     row_indices() { return m_row_indices.view(); }
    CBufferView<int>    row_indices() const { return m_row_indices.cview(); }
    BufferView<int>     col_indices() { return m_col_indices.view(); }
    CBufferView<int>    col_indices() const { return m_col_indices.cview(); }

    // indices are kept in lock-step with values by the assembler
    DeviceVector<int>&   row_index_buffer() { return m_row_indices; }
    DeviceVector<int>&   col_index_buffer() { return m_col_indices; }
    DeviceVector<ValueT> values_buffer() { return m_values; }

  private:
    int                 m_rows = 0, m_cols = 0;
    DeviceVector<ValueT> m_values;
    DeviceVector<int>    m_row_indices;
    DeviceVector<int>    m_col_indices;
};

// BCOO is a (block) triplet matrix that additionally exposes non_zeros().
template <typename T, int M = 1, int N = 1>
class DeviceBCOOMatrix : public DeviceTripletMatrix<T, M, N>
{
};

// ---------------------------------------------------------------------------
// Dense vector of scalars or fixed-size Eigen segments.
// ---------------------------------------------------------------------------
template <typename T>
class DeviceDenseVector
{
  public:
    using ValueT = T;
    void   resize(size_t n) { m_values.resize(n); }
    void   reserve(size_t n) { m_values.reserve(n); }
    size_t size() const { return m_values.size(); }
    size_t capacity() const { return m_values.capacity(); }

    BufferView<T>  view() { return m_values.view(); }
    CBufferView<T> cview() const { return m_values.cview(); }
    BufferView<T>  buffer_view() { return m_values.view(); }
    BufferView<T>  viewer() { return view(); }
    CBufferView<T> cviewer() const { return cview(); }

    DeviceDenseVector& operator=(const DeviceDenseVector& o)
    {
        m_values.copy_from(o.m_values);  // deep copy (PCG p = z)
        return *this;
    }
    void copy_from(const DeviceDenseVector& o) { m_values.copy_from(o.m_values); }

  private:
    DeviceVector<T> m_values;
};

// ---------------------------------------------------------------------------
// Doublet (sparse-segment) vector and BSR block matrix: only used by the
// directory-external converter paths; provided for completeness.
// ---------------------------------------------------------------------------
template <typename T, int N = 1>
class DeviceDoubletVector
{
  public:
    using ValueT   = std::conditional_t<(N == 1), T, Eigen::Vector<T, N>>;
    using DoubletT = VectorDoublet<T, N>;
    void   reshape(int num_segment) { m_count = num_segment; }
    void   resize_doublets(size_t nnz) { m_values.resize(nnz); }
    int    count() const { return m_count; }
    size_t doublet_count() const { return m_values.size(); }
    BufferView<ValueT> values() { return m_values.view(); }
    BufferView<int>    indices() { return m_indices.view(); }

  private:
    int                 m_count = 0;
    DeviceVector<ValueT> m_values;
    DeviceVector<int>    m_indices;
};

template <typename T, int N = 1>
class DeviceBSRMatrix
{
  public:
    using ValueT = std::conditional_t<(N == 1), T, Eigen::Matrix<T, N, N>>;
    void reshape(int row, int col)
    {
        m_rows = row;
        m_cols = col;
    }
    void resize(int non_zero_blocks) { m_values.resize(non_zero_blocks); }
    BufferView<int>    row_offsets() { return m_row_offsets.view(); }
    BufferView<int>    col_indices() { return m_col_indices.view(); }
    BufferView<ValueT> values() { return m_values.view(); }
    int                rows() const { return m_rows; }
    int                cols() const { return m_cols; }
    int                non_zeros() const { return (int)m_values.size(); }

  private:
    int                  m_rows = 0, m_cols = 0;
    DeviceVector<ValueT> m_values;
    DeviceVector<int>    m_row_offsets;
    DeviceVector<int>    m_col_indices;
};

// ---------------------------------------------------------------------------
// Reductions replacing muda::LinearSystemContext::dot / norm.
// Device-side cub reduction into a temporary DeviceVar, synced back to host.
// ---------------------------------------------------------------------------
template <typename T>
T dot(CBufferView<T> x, CBufferView<T> y, cudaStream_t s = default_stream())
{
    // element-wise product then sum
    DeviceVector<T> tmp(x.size());
    parallel_for(
        (int)x.size(), [x, y, t = tmp.viewer()] __device__(int i) mutable
        { t[i] = x[i] * y[i]; },
        s);
    DeviceVar<T> result;
    DeviceReduce(s).Sum(tmp.cview().data(), result.data(), (int)tmp.size());
    T host;
    result.copy_to(host, s);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
    return host;
}

template <typename T>
T norm(CBufferView<T> x, cudaStream_t s = default_stream())
{
    return std::sqrt(dot(x, x, s));
}

// ---------------------------------------------------------------------------
// LinearSystemContext: holds a cublas handle and provides dot/norm reductions
// on dense vectors, matching muda::LinearSystemContext's used surface.
// ---------------------------------------------------------------------------
class LinearSystemContext
{
  public:
    LinearSystemContext(cudaStream_t s = default_stream())
        : m_stream(s)
    {
        cublasCreate(&m_handle);
        cublasSetStream(m_handle, m_stream);
    }
    ~LinearSystemContext()
    {
        if(m_handle)
            cublasDestroy(m_handle);
    }
    LinearSystemContext(const LinearSystemContext&)            = delete;
    LinearSystemContext& operator=(const LinearSystemContext&) = delete;

    // dense dot: result = sum(x[i] * y[i])
    template <typename T>
    T dot(const DeviceDenseVector<T>& x, const DeviceDenseVector<T>& y)
    {
        return dot_view(x.cview(), y.cview());
    }
    template <typename T>
    T dot(CBufferView<T> x, CBufferView<T> y)
    {
        return dot_view(x, y);
    }

    // dense 2-norm
    template <typename T>
    T norm(const DeviceDenseVector<T>& x)
    {
        return norm_view(x.cview());
    }
    template <typename T>
    T norm(CBufferView<T> x)
    {
        return norm_view(x.cview());
    }

  private:
    template <typename T>
    T dot_view(CBufferView<T> x, CBufferView<T> y);
    template <typename T>
    T norm_view(CBufferView<T> x);

    cudaStream_t   m_stream;
    cublasHandle_t m_handle = nullptr;
};

template <>
inline float LinearSystemContext::dot_view<float>(CBufferView<float> x, CBufferView<float> y)
{
    float r;
    cublasSdot(m_handle, (int)x.size(), x.data(), 1, y.data(), 1, &r);
    return r;
}
template <>
inline double LinearSystemContext::dot_view<double>(CBufferView<double> x, CBufferView<double> y)
{
    double r;
    cublasDdot(m_handle, (int)x.size(), x.data(), 1, y.data(), 1, &r);
    return r;
}
template <>
inline float LinearSystemContext::norm_view<float>(CBufferView<float> x)
{
    float r;
    cublasSnrm2(m_handle, (int)x.size(), x.data(), 1, &r);
    return r;
}
template <>
inline double LinearSystemContext::norm_view<double>(CBufferView<double> x)
{
    double r;
    cublasDnrm2(m_handle, (int)x.size(), x.data(), 1, &r);
    return r;
}
}  // namespace uipc::backend::cuda_tool
