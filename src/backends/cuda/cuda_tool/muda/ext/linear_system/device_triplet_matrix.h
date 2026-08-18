#pragma once
#include <muda/buffer/device_buffer.h>
#include <muda/ext/linear_system/triplet_matrix_view.h>

namespace muda::details
{
template <typename T, int M, int N>
class MatrixFormatConverter;
}

namespace muda
{
template <typename T, int M, int N = M>
class DeviceTripletMatrix
{
  public:
    template <typename U, int M_, int N_>
    friend class details::MatrixFormatConverter;

    static constexpr bool IsBlockMatrix = (M > 1 || N > 1);
    using ValueT = std::conditional_t<IsBlockMatrix, Eigen::Matrix<T, M, N>, T>;


  protected:
    DeviceBuffer<ValueT> m_values;
    DeviceBuffer<int>    m_row_indices;
    DeviceBuffer<int>    m_col_indices;

    int m_rows = 0;
    int m_cols = 0;

  public:
    DeviceTripletMatrix()                                      = default;
    ~DeviceTripletMatrix()                                     = default;
    DeviceTripletMatrix(const DeviceTripletMatrix&)            = default;
    DeviceTripletMatrix(DeviceTripletMatrix&&)                 = default;
    DeviceTripletMatrix& operator=(const DeviceTripletMatrix&) = default;
    DeviceTripletMatrix& operator=(DeviceTripletMatrix&&)      = default;

    void reshape(int row, int col)
    {
        m_rows = row;
        m_cols = col;
    }

    void resize_triplets(size_t nonzero_count)
    {
        m_values.resize(nonzero_count);
        m_row_indices.resize(nonzero_count);
        m_col_indices.resize(nonzero_count);
    }

    void reserve_triplets(size_t nonzero_count)
    {
        m_values.reserve(nonzero_count);
        m_row_indices.reserve(nonzero_count);
        m_col_indices.reserve(nonzero_count);
    }

    void resize(int row, int col, size_t nonzero_count)
    {
        reshape(row, col);
        resize_triplets(nonzero_count);
    }

    static constexpr int block_dim() { return N; }

    auto values() { return m_values.view(); }
    auto values() const { return m_values.view(); }
    auto row_indices() { return m_row_indices.view(); }
    auto row_indices() const { return m_row_indices.view(); }
    auto col_indices() { return m_col_indices.view(); }
    auto col_indices() const { return m_col_indices.view(); }

    auto rows() const { return m_rows; }
    auto cols() const { return m_cols; }
    auto triplet_count() const { return m_values.size(); }
    auto triplet_capacity() const { return m_values.capacity(); }

    auto view()
    {
        return TripletMatrixView<T, M, N>{m_rows,
                                          m_cols,
                                          (int)m_values.size(),
                                          m_row_indices.data(),
                                          m_col_indices.data(),
                                          m_values.data()};
    }

    auto view() const { return remove_const(*this).view().as_const(); }

    auto cview() const { return view(); }

    auto viewer() { return view().viewer(); }

    auto cviewer() const { return view().cviewer(); }

    operator TripletMatrixView<T, M, N>() { return view(); }
    operator CTripletMatrixView<T, M, N>() const { return view(); }

    void clear()
    {
        m_rows = 0;
        m_cols = 0;
        m_values.clear();
        m_row_indices.clear();
        m_col_indices.clear();
    }
};
}  // namespace muda
#include "details/device_triplet_matrix.inl"
