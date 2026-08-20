#pragma once
// Minimal linear-system formats used by the backend's global linear solver.
// Raw CUDA containers + views; dot/norm reductions replace muda's cublas context.
#include <cuda_tool/buffer.h>
#include <cuda_tool/linear_system/views.h>
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
// Member surface aligned with cuda_tool::DeviceTripletMatrix.
// ---------------------------------------------------------------------------
template <typename T, int M = 1, int N = M>
class DeviceTripletMatrix
{
  public:
    static constexpr bool IsBlockMatrix = (M > 1 || N > 1);
    using ValueT   = std::conditional_t<IsBlockMatrix, Eigen::Matrix<T, M, N>, T>;
    using TripletT = MatrixTriplet<T, M, N>;

    void reshape(int rows, int cols)
    {
        m_rows = rows;
        m_cols = cols;
    }
    void resize_triplets(size_t n)
    {
        // indices are kept in lock-step with values
        m_values.resize(n);
        m_row_indices.resize(n);
        m_col_indices.resize(n);
    }
    void reserve_triplets(size_t n)
    {
        m_values.reserve(n);
        m_row_indices.reserve(n);
        m_col_indices.reserve(n);
    }
    void resize(int rows, int cols, size_t n)
    {
        reshape(rows, cols);
        resize_triplets(n);
    }
    void clear()
    {
        m_rows = m_cols = 0;
        m_values.clear();
        m_row_indices.clear();
        m_col_indices.clear();
    }

    static constexpr int block_dim() { return N; }

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

    TripletMatrixView<T, M, N> view()
    {
        return TripletMatrixView<T, M, N>{m_rows,
                                          m_cols,
                                          (int)m_values.size(),
                                          m_row_indices.data(),
                                          m_col_indices.data(),
                                          m_values.data()};
    }
    CTripletMatrixView<T, M, N> view() const
    {
        return const_cast<DeviceTripletMatrix*>(this)->view().as_const();
    }
    CTripletMatrixView<T, M, N> cview() const { return view(); }

    TripletMatrixViewer<T, M, N>  viewer() { return view(); }
    CTripletMatrixViewer<T, M, N> cviewer() const { return view(); }

    operator TripletMatrixView<T, M, N>() { return view(); }
    operator CTripletMatrixView<T, M, N>() const { return view(); }

    // raw owning-buffer access (cuda_tool extension, not in muda)
    DeviceVector<int>&   row_index_buffer() { return m_row_indices; }
    DeviceVector<int>&   col_index_buffer() { return m_col_indices; }
    DeviceVector<ValueT> values_buffer() { return m_values; }

  protected:
    int                  m_rows = 0, m_cols = 0;
    DeviceVector<ValueT> m_values;
    DeviceVector<int>    m_row_indices;
    DeviceVector<int>    m_col_indices;
};

// BCOO is a (block) triplet matrix that additionally exposes non_zeros()
// (non_zeros() is already provided by the base; kept for muda parity).
template <typename T, int M = 1, int N = M>
class DeviceBCOOMatrix : public DeviceTripletMatrix<T, M, N>
{
};

// scalar COO matrix (muda's DeviceCOOMatrix, without the cusparse descriptors)
template <typename T>
using DeviceCOOMatrix = DeviceBCOOMatrix<T, 1, 1>;

// ---------------------------------------------------------------------------
// Dense vector of scalars. Member surface aligned with cuda_tool::DeviceDenseVector
// (without the cusparse descriptor); view()/cview() hand out DenseVectorView.
// ---------------------------------------------------------------------------
template <typename T>
class DeviceDenseVector
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "only real numbers are supported");

  public:
    using ValueT = T;

    DeviceDenseVector() = default;
    DeviceDenseVector(DeviceDenseVector&&)            = default;
    DeviceDenseVector& operator=(DeviceDenseVector&&) = default;
    DeviceDenseVector(const DeviceDenseVector& o) { copy_from(o); }

    void   resize(size_t n) { m_values.resize(n); }
    void   reserve(size_t n) { m_values.reserve(n); }
    size_t size() const { return m_values.size(); }
    size_t capacity() const { return m_values.capacity(); }
    void   fill(const T& value) { m_values.fill(value); }

    DenseVectorView<T> view()
    {
        return DenseVectorView<T>{m_values.data(), 0, (int)m_values.size(),
                                  (int)m_values.size()};
    }
    CDenseVectorView<T> view() const
    {
        return const_cast<DeviceDenseVector*>(this)->view().as_const();
    }
    CDenseVectorView<T> cview() const { return view(); }

    DenseVectorViewer<T>  viewer() { return view(); }
    CDenseVectorViewer<T> viewer() const { return view(); }
    CDenseVectorViewer<T> cviewer() const { return view(); }

    BufferView<T>  buffer_view() { return m_values.view(); }
    CBufferView<T> buffer_view() const { return m_values.cview(); }

    operator DenseVectorView<T>() { return view(); }
    operator CDenseVectorView<T>() const { return view(); }

    DeviceDenseVector& operator=(const DeviceDenseVector& o)
    {
        m_values.copy_from(o.m_values);  // deep copy (PCG p = z)
        return *this;
    }
    void copy_from(const DeviceDenseVector& o) { m_values.copy_from(o.m_values); }

    // download to host (muda parity)
    void copy_to(T* host, cudaStream_t s = default_stream()) const
    {
        m_values.copy_to(host, s);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
    }
    template <typename Alloc>
    void copy_to(std::vector<T, Alloc>& host, cudaStream_t s = default_stream()) const
    {
        host.resize(m_values.size());
        copy_to(host.data(), s);
    }

  private:
    DeviceVector<T> m_values;
};

// ---------------------------------------------------------------------------
// Doublet (sparse-segment) vector. Member surface aligned with
// cuda_tool::DeviceDoubletVector.
// ---------------------------------------------------------------------------
template <typename T, int N = 1>
class DeviceDoubletVector
{
  public:
    using ValueT   = std::conditional_t<(N == 1), T, Eigen::Vector<T, N>>;
    using DoubletT = VectorDoublet<T, N>;
    static constexpr bool IsSegmentVector = (N > 1);

    void reshape(int num_segment) { m_count = num_segment; }
    void resize_doublets(size_t nnz)
    {
        // indices are kept in lock-step with values
        m_values.resize(nnz);
        m_indices.resize(nnz);
    }
    void reserve_doublets(size_t nnz)
    {
        m_values.reserve(nnz);
        m_indices.reserve(nnz);
    }
    void resize(int num_segment, size_t nnz)
    {
        reshape(num_segment);
        resize_doublets(nnz);
    }
    void clear()
    {
        m_count = 0;
        m_values.clear();
        m_indices.clear();
    }

    int    count() const { return m_count; }
    size_t doublet_count() const { return m_values.size(); }
    size_t doublet_capacity() const { return m_values.capacity(); }

    BufferView<ValueT>  values() { return m_values.view(); }
    CBufferView<ValueT> values() const { return m_values.cview(); }
    BufferView<int>     indices() { return m_indices.view(); }
    CBufferView<int>    indices() const { return m_indices.cview(); }

    DoubletVectorView<T, N> view()
    {
        return DoubletVectorView<T, N>{m_count, (int)m_values.size(),
                                       m_indices.data(), m_values.data()};
    }
    CDoubletVectorView<T, N> view() const
    {
        return const_cast<DeviceDoubletVector*>(this)->view().as_const();
    }
    CDoubletVectorView<T, N> cview() const { return view(); }

    DoubletVectorViewer<T, N>  viewer() { return view(); }
    CDoubletVectorViewer<T, N> viewer() const { return view(); }
    CDoubletVectorViewer<T, N> cviewer() const { return view(); }

    operator DoubletVectorView<T, N>() { return view(); }
    operator CDoubletVectorView<T, N>() const { return view(); }

  protected:
    int                  m_count = 0;
    DeviceVector<ValueT> m_values;
    DeviceVector<int>    m_indices;
};

// BCOO vector is a doublet vector that additionally exposes non_zeros()
// (muda's DeviceBCOOVector, without the cusparse descriptor).
template <typename T, int N>
class DeviceBCOOVector : public DeviceDoubletVector<T, N>
{
  public:
    auto non_zeros() const { return this->m_values.size(); }
};

// BSR block matrix: only used by the converter paths; provided for
// completeness. Semantics follow muda: reshape() allocates row_offsets
// (rows + 1), resize() only allocates values + col_indices.
template <typename T, int N = 1>
class DeviceBSRMatrix
{
  public:
    using ValueT = std::conditional_t<(N == 1), T, Eigen::Matrix<T, N, N>>;
    static constexpr bool IsBlockMatrix = (N > 1);

    void reshape(int row, int col)
    {
        m_rows = row;
        m_cols = col;
        m_row_offsets.resize(row + 1);
    }
    void resize(int non_zero_blocks)
    {
        m_values.resize(non_zero_blocks);
        m_col_indices.resize(non_zero_blocks);
    }
    void reserve(int non_zero_blocks)
    {
        m_values.reserve(non_zero_blocks);
        m_col_indices.reserve(non_zero_blocks);
    }
    void reserve_offsets(int size) { m_row_offsets.reserve(size); }
    void clear()
    {
        m_rows = m_cols = 0;
        m_values.clear();
        m_row_offsets.clear();
        m_col_indices.clear();
    }

    static constexpr int block_size() { return N; }

    BufferView<int>     row_offsets() { return m_row_offsets.view(); }
    CBufferView<int>    row_offsets() const { return m_row_offsets.cview(); }
    BufferView<int>     col_indices() { return m_col_indices.view(); }
    CBufferView<int>    col_indices() const { return m_col_indices.cview(); }
    BufferView<ValueT>  values() { return m_values.view(); }
    CBufferView<ValueT> values() const { return m_values.cview(); }
    int                 rows() const { return m_rows; }
    int                 cols() const { return m_cols; }
    int                 non_zeros() const { return (int)m_values.size(); }

  private:
    int                  m_rows = 0, m_cols = 0;
    DeviceVector<ValueT> m_values;
    DeviceVector<int>    m_row_offsets;
    DeviceVector<int>    m_col_indices;
};

// ---------------------------------------------------------------------------
// Dense matrix on device (row-major, contiguous). Minimal port of
// cuda_tool::DeviceDenseMatrix: the backend only declares it as a debug-dump
// buffer, so the DenseMatrixView/Viewer layer is not ported.
// ---------------------------------------------------------------------------
template <typename T>
class DeviceDenseMatrix
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "only real numbers are supported");

  public:
    using value_type = T;

    DeviceDenseMatrix() = default;
    DeviceDenseMatrix(size_t row, size_t col, bool sym = false)
        : m_sym(sym)
    {
        reshape(row, col);
    }

    void reshape(size_t row, size_t col)
    {
        m_row = row;
        m_col = col;
        m_data.resize(Extent2D{row, col});
    }
    void fill(T value) { BufferLaunch().fill(m_data.view(), value); }
    void clear() { m_data.clear(); }

    size_t row() const { return m_row; }
    size_t col() const { return m_col; }
    void   sym(bool sym = true) { m_sym = sym; }
    bool   sym() const { return m_sym; }

    Buffer2DView<T>  buffer_view() { return m_data.view(); }
    CBuffer2DView<T> buffer_view() const { return m_data.cview(); }

  private:
    DeviceBuffer2D<T> m_data;
    size_t            m_row = 0;
    size_t            m_col = 0;
    bool              m_sym = false;
};

// ---------------------------------------------------------------------------
// LinearSystemContext: holds a cublas handle and provides dot/norm reductions
// on dense vectors, matching cuda_tool::LinearSystemContext's used surface.
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
        return dot(x.cview(), y.cview());
    }
    template <typename T>
    T dot(CBufferView<T> x, CBufferView<T> y)
    {
        return dot_view(x, y);
    }
    template <typename T>
    T dot(CDenseVectorView<T> x, CDenseVectorView<T> y)
    {
        return dot_view(x.buffer_view(), y.buffer_view());
    }

    // dense 2-norm
    template <typename T>
    T norm(const DeviceDenseVector<T>& x)
    {
        return norm(x.cview());
    }
    template <typename T>
    T norm(CBufferView<T> x)
    {
        return norm_view(x);
    }
    template <typename T>
    T norm(CDenseVectorView<T> x)
    {
        return norm_view(x.buffer_view());
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
