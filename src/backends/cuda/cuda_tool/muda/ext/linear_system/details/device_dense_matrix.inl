#include <cublas_v2.h>

namespace muda
{
template <typename Ty>
DeviceDenseMatrix<Ty>::DeviceDenseMatrix(size_t row, size_t col, bool sym)
    : m_row{row}
    , m_col{col}
    , m_sym{sym}
    , m_data{muda::Extent2D{col, row}}
{
}
template <typename Ty>
DeviceDenseMatrix<Ty>::DeviceDenseMatrix(DeviceDenseMatrix&& other)
    : m_row{other.m_row}
    , m_col{other.m_col}
    , m_data{std::move(other.m_data)}
{
    other.m_row = 0;
    other.m_col = 0;
}
template <typename Ty>
DeviceDenseMatrix<Ty>& DeviceDenseMatrix<Ty>::operator=(DeviceDenseMatrix&& other)
{
    if(this != &other)
    {
        m_row       = other.m_row;
        m_col       = other.m_col;
        m_data      = std::move(other.m_data);
        other.m_row = 0;
        other.m_col = 0;
    }
    return *this;
}

template <typename Ty>
void DeviceDenseMatrix<Ty>::reshape(size_t row, size_t col)
{
    m_data.resize(muda::Extent2D{col, row});
    m_row = row;
    m_col = col;
}

template <typename Ty>
void DeviceDenseMatrix<Ty>::fill(Ty value)
{
    m_data.fill(value);
}

template <typename Ty>
void DeviceDenseMatrix<Ty>::copy_to(Eigen::MatrixX<Ty>& mat) const
{
    std::vector<Ty> host_data;
    m_data.copy_to(host_data);
    mat.resize(m_row, m_col);

    for(size_t i = 0; i < m_row; ++i)
    {
        for(size_t j = 0; j < m_col; ++j)
        {
            mat(i, j) = host_data[j * m_row + i];
        }
    }
}
template <typename Ty>
void DeviceDenseMatrix<Ty>::copy_to(std::vector<Ty>& vec) const
{
    m_data.copy_to(vec);
}
template <typename Ty>
DeviceDenseMatrix<Ty>::DeviceDenseMatrix(const Eigen::MatrixX<Ty>& mat)
{
    reshape(mat.rows(), mat.cols());
    std::vector<Ty> host_data(m_row * m_col);

    for(size_t i = 0; i < m_row; ++i)
    {
        for(size_t j = 0; j < m_col; ++j)
        {
            host_data[j * m_row + i] = mat(i, j);
        }
    }
    m_data.copy_from(host_data);
}
template <typename Ty>
DeviceDenseMatrix<Ty>& DeviceDenseMatrix<Ty>::operator=(const Eigen::MatrixX<Ty>& mat)
{
    if(mat.rows() != m_row || mat.cols() != m_col)
    {
        reshape(mat.rows(), mat.cols());
    }
    std::vector<Ty> host_data(m_row * m_col);

    for(size_t i = 0; i < m_row; ++i)
    {
        for(size_t j = 0; j < m_col; ++j)
        {
            host_data[j * m_row + i] = mat(i, j);
        }
    }

    m_data.copy_from(host_data);
    return *this;
}
template <typename Ty>
DenseMatrixView<Ty> DeviceDenseMatrix<Ty>::T()
{
    return DenseMatrixView{m_data, m_row, m_col, true, m_sym};
}

template <typename Ty>
CDenseMatrixView<Ty> DeviceDenseMatrix<Ty>::T() const
{
    return CDenseMatrixView{m_data, m_row, m_col, true, m_sym};
}

template <typename Ty>
DenseMatrixView<Ty> DeviceDenseMatrix<Ty>::view()
{
    return DenseMatrixView<Ty>{m_data.view(), m_row, m_col, false, m_sym};
}

template <typename Ty>
CDenseMatrixView<Ty> DeviceDenseMatrix<Ty>::view() const
{
    return CDenseMatrixView<Ty>{m_data.view(), m_row, m_col, false, m_sym};
}

template <typename Ty>
DeviceDenseMatrix<Ty>::operator CDenseMatrixView<Ty>() const
{
    return view();
}
template <typename Ty>
DeviceDenseMatrix<Ty>::operator DenseMatrixView<Ty>()
{
    return view();
}
}  // namespace muda
