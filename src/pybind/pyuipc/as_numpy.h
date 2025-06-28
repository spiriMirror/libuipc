#pragma once
#include <pyuipc/pyuipc.h>
#include <pybind11/numpy.h>
#include <uipc/common/span.h>
#include <uipc/common/log.h>
#include <Eigen/Core>

namespace pyuipc
{
namespace py = pybind11;
using namespace uipc;

inline void set_read_write_flags(py::array& arr, bool readonly)
{
    if(readonly)
        py::detail::array_proxy(arr.ptr())->flags &= ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
    else
        py::detail::array_proxy(arr.ptr())->flags |= py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
}

template <typename T>
auto buffer_info(const span<T>& s)
{
    // must use non-const type for buffer_info
    // otherwise, undefined behavior will occur
    using RawT             = std::remove_const_t<T>;
    constexpr bool IsConst = std::is_const_v<T>;

    py::buffer_info info;
    info.ndim     = 1;
    info.shape    = {(py::ssize_t)s.size()};
    info.strides  = {(py::ssize_t)sizeof(RawT)};
    info.format   = py::format_descriptor<RawT>::format();
    info.itemsize = sizeof(RawT);
    info.ptr      = (void*)s.data();
    info.readonly = IsConst;

    return info;
}

template <typename T>
auto as_numpy(const span<T>& s, py::handle obj)
{
    auto arr = py::array_t<T, py::array::c_style>(buffer_info(s), obj);

    // NOTE:
    // In some impl, the py-array may own the data when the span is empty.
    // To avoid a fake assertion, we allow the array to own the data when the span is empty.
    PYUIPC_ASSERT(arr.size() == 0 || !arr.owndata(),
                  "the array must share the data with the input span");

    set_read_write_flags(arr, std::is_const_v<T>);
    PYUIPC_ASSERT(arr.writeable() == !std::is_const_v<T>,
                  "writeable flag must be consistent with the constness of the span");
    return arr;
}

template <typename T>
span<T> as_span(py::array_t<T> arr)
    requires std::is_arithmetic_v<T>
{
    PYUIPC_ASSERT(arr.ndim() == 1, "array must be 1D, yours={}", arr.ndim());

    if constexpr(std::is_const_v<T>)
        return span<T>(arr.data(), arr.size());
    else
        return span<T>(arr.mutable_data(), arr.size());
}

template <typename T, int M, int N, int Options>
auto buffer_info(const span<const Eigen::Matrix<T, M, N, Options>>& v)
{
    using Matrix = Eigen::Matrix<T, M, N, Options>;
    // view it as a 3-order tensor
    py::ssize_t stride_2 = Matrix::OuterStrideAtCompileTime;
    py::ssize_t stride_3 = Matrix::InnerStrideAtCompileTime;

    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;

    if(!rowMajor)
        std::swap(stride_2, stride_3);

    py::buffer_info info;
    info.ndim     = 3;
    info.shape    = {(py::ssize_t)v.size(), M, N};
    info.strides  = {(py::ssize_t)sizeof(Matrix),
                     stride_2 * (py::ssize_t)sizeof(T),
                     stride_3 * (py::ssize_t)sizeof(T)};
    info.format   = py::format_descriptor<T>::format();
    info.itemsize = sizeof(T);
    info.ptr      = (void*)v.data();
    info.readonly = true;
    return info;
}

template <typename T, int M, int N, int Options>
auto buffer_info(const span<Eigen::Matrix<T, M, N, Options>>& v)
{
    using Matrix = Eigen::Matrix<T, M, N, Options>;
    // view it as a 3-order tensor
    py::ssize_t stride_2 = Matrix::OuterStrideAtCompileTime;
    py::ssize_t stride_3 = Matrix::InnerStrideAtCompileTime;

    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;

    if(!rowMajor)
        std::swap(stride_2, stride_3);

    py::buffer_info info;
    info.ndim     = 3;
    info.shape    = {(py::ssize_t)v.size(), M, N};
    info.strides  = {(py::ssize_t)sizeof(Matrix),
                     stride_2 * (py::ssize_t)sizeof(T),
                     stride_3 * (py::ssize_t)sizeof(T)};
    info.format   = py::format_descriptor<T>::format();
    info.itemsize = sizeof(T);
    info.ptr      = (void*)v.data();
    info.readonly = false;
    return info;
}

template <typename T, int M, int N, int Options>
auto as_numpy(const span<Eigen::Matrix<T, M, N, Options>>& v, py::handle obj)
    requires(M > 0 && N > 0)
{
    auto arr = py::array_t<T, py::array::c_style>(buffer_info(v), obj);
    PYUIPC_ASSERT(!arr.owndata() || v.size() == 0,
                  "the array must share the data with the input span");

    set_read_write_flags(arr, false);
    PYUIPC_ASSERT(arr.writeable(), "writeable flag must be true");

    return arr;
}

template <typename T, int M, int N, int Options>
auto as_numpy(const span<const Eigen::Matrix<T, M, N, Options>>& v, py::handle obj)
    requires(M > 0 && N > 0)
{
    auto arr = py::array_t<T, py::array::c_style>(buffer_info(v), obj);
    PYUIPC_ASSERT(!arr.owndata() || v.size() == 0,
                  "the array must share the data with the input span");
    return arr;
}

template <typename MatrixT>
span<MatrixT> as_span_of(py::array_t<typename MatrixT::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    constexpr int Rows = MatrixT::RowsAtCompileTime;
    constexpr int Cols = MatrixT::ColsAtCompileTime;

    constexpr bool IsConst = std::is_const_v<MatrixT>;

    if(arr.ndim() == 2)
    {
        if(Rows == 1 || Cols == 1)
        {
            PYUIPC_ASSERT(arr.shape(1) == Rows * Cols,
                          "Shape mismatch, ask for shape=(N,{}), yours=({},{})",
                          Rows * Cols,
                          arr.shape(0),
                          arr.shape(1));
        }
        else
        {
            throw PyException(PYUIPC_MSG("array must be 3D"));
        }
    }
    else if(arr.ndim() == 3)
    {
        PYUIPC_ASSERT(arr.shape(1) == Rows && arr.shape(2) == Cols,
                      "Shape mismatch, ask for shape=(N,{},{}), yours=({},{},{})",
                      Rows,
                      Cols,
                      arr.shape(0),
                      arr.shape(1),
                      arr.shape(2));
    }
    else
    {
        throw PyException(PYUIPC_MSG("array must be 2D or 3D, yours={}", arr.ndim()));
    }

    if constexpr(IsConst)
        return span<MatrixT>((MatrixT*)arr.data(), arr.shape(0));
    else
        return span<MatrixT>((MatrixT*)arr.mutable_data(), arr.shape(0));
}

template <typename MatrixT>
bool is_span_of(py::array_t<typename MatrixT::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    constexpr int Rows = MatrixT::RowsAtCompileTime;
    constexpr int Cols = MatrixT::ColsAtCompileTime;

    constexpr bool IsConst = std::is_const_v<MatrixT>;

    if(arr.ndim() == 2)
    {
        if constexpr(Rows == 1 || Cols == 1)
        {
            if(arr.shape(1) != Rows * Cols)
            {
                return false;  // Shape mismatch
            }
        }
        else
        {
            return false;
        }
    }
    else if(arr.ndim() == 3)
    {
        if(!(arr.shape(1) == Rows && arr.shape(2) == Cols))
        {
            return false;  // Shape mismatch
        }
    }
    else
    {
        return false;  // Not 2D or 3D
    }

    return true;
}

template <typename T, int M, int N, int Options>
auto buffer_info(const Matrix<T, M, N, Options>& m)
    requires(M > 0 && N > 0)
{
    using Matrix         = Eigen::Matrix<T, M, N, Options>;
    py::ssize_t stride_1 = Matrix::OuterStrideAtCompileTime;
    py::ssize_t stride_2 = Matrix::InnerStrideAtCompileTime;

    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;

    if(!rowMajor)
        std::swap(stride_1, stride_2);

    py::buffer_info info;
    info.ndim  = 2;
    info.shape = {M, N};
    info.strides = {stride_1 * (py::ssize_t)sizeof(T), stride_2 * (py::ssize_t)sizeof(T)};
    info.format   = py::format_descriptor<T>::format();
    info.itemsize = sizeof(T);
    info.ptr      = (void*)m.data();
    info.readonly = true;
    return info;
};

template <typename T, int M, int N, int Options>
auto buffer_info(Matrix<T, M, N, Options>& m)
    requires(M > 0 && N > 0)
{
    auto info     = buffer_info(std::as_const(m));
    info.readonly = false;
    return info;
};

template <typename T, int M, int N, int Options>
auto as_numpy(const Matrix<T, M, N, Options>& m)
    requires(M > 0 && N > 0)
{
    auto arr = py::array_t<T, py::array::c_style>(buffer_info(m));
    PYUIPC_ASSERT(arr.owndata(), "the array must own the data");
    return arr;
}

template <typename T, int M, int N, int Options>
auto as_numpy(Matrix<T, M, N, Options>& m)
    requires(M > 0 && N > 0)
{
    auto arr = py::array_t<T, py::array::c_style>(buffer_info(m));
    PYUIPC_ASSERT(arr.owndata(), "the array must own the data");
    return arr;
}

template <typename MatrixT>
MatrixT to_matrix(py::array_t<typename MatrixT::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    constexpr int Rows = MatrixT::RowsAtCompileTime;
    constexpr int Cols = MatrixT::ColsAtCompileTime;

    MatrixT m;

    if(arr.ndim() == 1)
    {
        if(Rows == 1 || Cols == 1)
        {
            PYUIPC_ASSERT(arr.size() == Rows * Cols,
                          "Shape mismatch, ask for shape=(N,{}), yours={}",
                          Rows * Cols,
                          arr.size());
        }
        else
        {
            throw PyException(PYUIPC_MSG("array must be 2D, yours={}", arr.ndim()));
        }

        auto count = std::max(Rows, Cols);

        for(int i = 0; i < count; i++)
            m(i) = arr.at(i);
    }
    else if(arr.ndim() == 2)
    {
        PYUIPC_ASSERT(arr.shape(0) == Rows && arr.shape(1) == Cols,
                      "Shape mismatch, ask for shape=({},{}), yours=({},{})",
                      Rows,
                      Cols,
                      arr.shape(0),
                      arr.shape(1));

        for(int i = 0; i < Rows; i++)
            for(int j = 0; j < Cols; j++)
                m(i, j) = arr.at(i, j);
    }
    else
    {
        throw PyException(PYUIPC_MSG("array must be 1D or 2D, yours={}", arr.ndim()));
    }

    return m;
}
}  // namespace pyuipc
