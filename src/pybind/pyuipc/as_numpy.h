#pragma once
#include <pyuipc/exception.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <uipc/common/span.h>
#include <uipc/common/log.h>
#include <Eigen/Core>

namespace pyuipc
{
namespace py = nanobind;
using namespace uipc;

// Convenience type aliases for numpy arrays
template <typename T>
using NpArray = py::ndarray<py::numpy, T>;

using ssize_t = int64_t;

// Helper: create an owned numpy array (allocates new memory)
template <typename T>
NpArray<T> make_numpy(const std::vector<size_t>& shape)
{
    size_t total = 1;
    for(auto s : shape)
        total *= s;
    T* data = new T[total]();
    py::capsule owner(data, [](void* p) noexcept { delete[] static_cast<T*>(p); });
    return NpArray<T>(data, shape.size(), shape.data(), owner);
}

// ---- span<T> → numpy array (shares memory with Python handle as owner) ----

template <typename T>
NpArray<T> as_numpy(const span<T>& s, py::handle obj)
{
    using RawT = std::remove_const_t<T>;
    size_t shape[1] = {s.size()};
    return NpArray<T>(const_cast<RawT*>(s.data()), 1, shape, obj);
}

template <typename T>
span<T> as_span(NpArray<T> arr)
    requires std::is_arithmetic_v<T>
{
    PYUIPC_ASSERT(arr.ndim() == 1, "array must be 1D, yours={}", arr.ndim());
    return span<T>(arr.data(), arr.size());
}

// ---- span<Eigen::Matrix> → numpy array (3D: N×M×N) ----

template <typename T, int M, int N, int Options>
NpArray<T> as_numpy(const span<const Eigen::Matrix<T, M, N, Options>>& v, py::handle obj)
    requires(M > 0 && N > 0)
{
    size_t shape[3] = {v.size(), (size_t)M, (size_t)N};
    int64_t strides[3];
    using Matrix = Eigen::Matrix<T, M, N, Options>;
    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
    if(rowMajor)
    {
        strides[0] = (int64_t)sizeof(Matrix);
        strides[1] = (int64_t)(N * sizeof(T));
        strides[2] = (int64_t)sizeof(T);
    }
    else
    {
        strides[0] = (int64_t)sizeof(Matrix);
        strides[1] = (int64_t)sizeof(T);
        strides[2] = (int64_t)(M * sizeof(T));
    }
    return NpArray<T>(const_cast<T*>(reinterpret_cast<const T*>(v.data())),
                      3, shape, obj, strides);
}

template <typename T, int M, int N, int Options>
NpArray<T> as_numpy(const span<Eigen::Matrix<T, M, N, Options>>& v, py::handle obj)
    requires(M > 0 && N > 0)
{
    using Matrix = Eigen::Matrix<T, M, N, Options>;
    return as_numpy(span<const Eigen::Matrix<T, M, N, Options>>(v.data(), v.size()), obj);
}

// ---- as_span_of: ndarray → span<MatrixT> ----

template <typename MatrixT>
span<MatrixT> as_span_of(NpArray<typename MatrixT::Scalar> arr)
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
            PYUIPC_ASSERT((int64_t)arr.shape(1) == Rows * Cols,
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
        PYUIPC_ASSERT((int64_t)arr.shape(1) == Rows && (int64_t)arr.shape(2) == Cols,
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

    return span<MatrixT>(reinterpret_cast<MatrixT*>(arr.data()), arr.shape(0));
}

template <typename MatrixT>
bool is_span_of(NpArray<typename MatrixT::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    constexpr int Rows = MatrixT::RowsAtCompileTime;
    constexpr int Cols = MatrixT::ColsAtCompileTime;

    if(arr.ndim() == 2)
    {
        if constexpr(Rows == 1 || Cols == 1)
        {
            if((int64_t)arr.shape(1) != Rows * Cols)
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else if(arr.ndim() == 3)
    {
        if(!((int64_t)arr.shape(1) == Rows && (int64_t)arr.shape(2) == Cols))
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    return true;
}

// ---- Single Eigen::Matrix → owned numpy array ----

template <typename T, int M, int N, int Options>
NpArray<T> as_numpy(const Eigen::Matrix<T, M, N, Options>& m)
    requires(M > 0 && N > 0)
{
    using Matrix = Eigen::Matrix<T, M, N, Options>;
    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;

    size_t shape[2] = {(size_t)M, (size_t)N};
    int64_t strides[2];
    if(rowMajor)
    {
        strides[0] = (int64_t)(N * sizeof(T));
        strides[1] = (int64_t)sizeof(T);
    }
    else
    {
        strides[0] = (int64_t)sizeof(T);
        strides[1] = (int64_t)(M * sizeof(T));
    }
    // Return owned copy
    T* data = new T[M * N];
    std::copy(m.data(), m.data() + M * N, data);
    py::capsule owner(data, [](void* p) noexcept { delete[] static_cast<T*>(p); });
    return NpArray<T>(data, 2, shape, owner, strides);
}

template <typename T, int M, int N, int Options>
NpArray<T> as_numpy(Eigen::Matrix<T, M, N, Options>& m)
    requires(M > 0 && N > 0)
{
    return as_numpy(std::as_const(m));
}

// ---- to_matrix: ndarray → Eigen::Matrix ----

template <typename MatrixT>
MatrixT to_matrix(NpArray<typename MatrixT::Scalar> arr)
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
            PYUIPC_ASSERT((int64_t)arr.size() == Rows * Cols,
                          "Shape mismatch, ask for shape=(N,{}), yours={}",
                          Rows * Cols,
                          arr.size());
        }
        else
        {
            throw PyException(PYUIPC_MSG("array must be 2D, yours={}", arr.ndim()));
        }

        auto count    = std::max(Rows, Cols);
        auto* ptr     = arr.data();
        int64_t stride = arr.stride(0) / (int64_t)sizeof(typename MatrixT::Scalar);

        for(int i = 0; i < count; i++)
            m(i) = ptr[i * stride];
    }
    else if(arr.ndim() == 2)
    {
        PYUIPC_ASSERT((int64_t)arr.shape(0) == Rows && (int64_t)arr.shape(1) == Cols,
                      "Shape mismatch, ask for shape=({},{}), yours=({},{})",
                      Rows,
                      Cols,
                      arr.shape(0),
                      arr.shape(1));

        auto*   ptr        = arr.data();
        int64_t row_stride = arr.stride(0) / (int64_t)sizeof(typename MatrixT::Scalar);
        int64_t col_stride = arr.stride(1) / (int64_t)sizeof(typename MatrixT::Scalar);
        for(int i = 0; i < Rows; i++)
            for(int j = 0; j < Cols; j++)
                m(i, j) = ptr[i * row_stride + j * col_stride];
    }
    else
    {
        throw PyException(PYUIPC_MSG("array must be 1D or 2D, yours={}", arr.ndim()));
    }

    return m;
}
}  // namespace pyuipc
