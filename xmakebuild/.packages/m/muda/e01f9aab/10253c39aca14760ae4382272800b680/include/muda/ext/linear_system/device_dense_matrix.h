#pragma once
#include <muda/ext/linear_system/dense_matrix_view.h>
#include <muda/buffer/device_buffer_2d.h>

namespace muda::details
{
template <typename T, int M, int N>
class MatrixFormatConverter;
}

namespace muda
{
// A column major dense matrix on device.
template <typename Ty>
class DeviceDenseMatrix
{
    static_assert(std::is_same_v<Ty, float> || std::is_same_v<Ty, double>,
                  "now only support real number");

    DeviceBuffer2D<Ty> m_data;
    size_t             m_row = 0;
    size_t             m_col = 0;
    bool               m_sym = false;

  public:
    using value_type    = Ty;
    DeviceDenseMatrix() = default;
    DeviceDenseMatrix(size_t row, size_t col, bool sym = false);
    void sym(bool sym = true) { m_sym = sym; }
    bool sym() const { return m_sym; }

    DeviceDenseMatrix(const DeviceDenseMatrix&) = default;
    DeviceDenseMatrix(DeviceDenseMatrix&&);

    DeviceDenseMatrix& operator=(const DeviceDenseMatrix&) = default;
    DeviceDenseMatrix& operator=(DeviceDenseMatrix&&);

    size_t row() const { return m_row; }
    size_t col() const { return m_col; }

    void reshape(size_t row, size_t col);
    void fill(Ty value);
    void copy_to(Eigen::MatrixX<Ty>& mat) const;
    void copy_to(std::vector<Ty>& vec) const;

    DeviceDenseMatrix(const Eigen::MatrixX<Ty>&);
    DeviceDenseMatrix& operator=(const Eigen::MatrixX<Ty>&);

    DenseMatrixViewer<Ty>  viewer() { return view().viewer(); }
    CDenseMatrixViewer<Ty> cviewer() const { return view().cviewer(); }

    auto buffer_view() const { return m_data.view(); }
    auto buffer_view() { return m_data.view(); }

    DenseMatrixView<Ty>  T();
    CDenseMatrixView<Ty> T() const;

    DenseMatrixView<Ty>  view();
    CDenseMatrixView<Ty> view() const;
    CDenseMatrixView<Ty> cview() const { return view(); }

    operator DenseMatrixView<Ty>();
    operator CDenseMatrixView<Ty>() const;
};
}  // namespace muda

#include "details/device_dense_matrix.inl"
