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

// Convenience alias for numpy arrays
template <typename T>
using numpy_array = py::ndarray<py::numpy, std::remove_const_t<T>>;

// Helper: set writeable flag on a numpy array via Python attribute
inline void set_read_write_flags(py::object& arr, bool readonly)
{
    arr.attr("flags").attr("writeable") = py::bool_(!readonly);
}

// ============================================================================
// Expose span<T> as numpy array (zero-copy, shares memory)
// ============================================================================

template <typename T>
py::object as_numpy(const span<T>& s, py::handle obj)
{
    using RawT             = std::remove_const_t<T>;
    constexpr bool IsConst = std::is_const_v<T>;

    size_t  shape[1]   = {s.size()};
    int64_t strides[1] = {(int64_t)sizeof(RawT)};

    py::ndarray<py::numpy, RawT> nd(
        (void*)s.data(), 1, shape, obj, strides);

    py::object py_arr = py::cast(nd, py::rv_policy::none);

    if constexpr(IsConst)
        set_read_write_flags(py_arr, true);

    return py_arr;
}

// ============================================================================
// Create span<T> from numpy array
// ============================================================================

template <typename T>
span<T> as_span(numpy_array<T> arr)
    requires std::is_arithmetic_v<std::remove_const_t<T>>
{
    PYUIPC_ASSERT(arr.ndim() == 1, "array must be 1D, yours={}", arr.ndim());
    return span<T>((T*)arr.data(), arr.shape(0));
}

// ============================================================================
// Expose span<Eigen::Matrix> as 3D numpy array (zero-copy)
// ============================================================================

template <typename T, int M, int N, int Options>
py::object as_numpy(const span<Eigen::Matrix<T, M, N, Options>>& v, py::handle obj)
    requires(M > 0 && N > 0)
{
    using Matrix = Eigen::Matrix<T, M, N, Options>;

    int64_t stride_2 = Matrix::OuterStrideAtCompileTime;
    int64_t stride_3 = Matrix::InnerStrideAtCompileTime;

    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
    if(!rowMajor)
        std::swap(stride_2, stride_3);

    size_t  shape[3]   = {v.size(), (size_t)M, (size_t)N};
    int64_t strides[3] = {(int64_t)sizeof(Matrix),
                          stride_2 * (int64_t)sizeof(T),
                          stride_3 * (int64_t)sizeof(T)};

    py::ndarray<py::numpy, T> nd(
        (void*)v.data(), 3, shape, obj, strides);

    return py::cast(nd, py::rv_policy::none);
}

template <typename T, int M, int N, int Options>
py::object as_numpy(const span<const Eigen::Matrix<T, M, N, Options>>& v,
                    py::handle obj)
    requires(M > 0 && N > 0)
{
    using Matrix = Eigen::Matrix<T, M, N, Options>;

    int64_t stride_2 = Matrix::OuterStrideAtCompileTime;
    int64_t stride_3 = Matrix::InnerStrideAtCompileTime;

    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
    if(!rowMajor)
        std::swap(stride_2, stride_3);

    size_t  shape[3]   = {v.size(), (size_t)M, (size_t)N};
    int64_t strides[3] = {(int64_t)sizeof(Matrix),
                          stride_2 * (int64_t)sizeof(T),
                          stride_3 * (int64_t)sizeof(T)};

    py::ndarray<py::numpy, T> nd(
        (void*)v.data(), 3, shape, obj, strides);

    py::object py_arr = py::cast(nd, py::rv_policy::none);
    set_read_write_flags(py_arr, true);  // const span -> readonly
    return py_arr;
}

// ============================================================================
// Create span<Matrix> from numpy array
// ============================================================================

template <typename MatrixT>
span<MatrixT> as_span_of(numpy_array<typename std::remove_const_t<MatrixT>::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    constexpr int Rows = std::remove_const_t<MatrixT>::RowsAtCompileTime;
    constexpr int Cols = std::remove_const_t<MatrixT>::ColsAtCompileTime;

    if(arr.ndim() == 2)
    {
        if(Rows == 1 || Cols == 1)
        {
            PYUIPC_ASSERT((size_t)arr.shape(1) == (size_t)(Rows * Cols),
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
        PYUIPC_ASSERT((size_t)arr.shape(1) == (size_t)Rows
                          && (size_t)arr.shape(2) == (size_t)Cols,
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

    return span<MatrixT>((MatrixT*)arr.data(), arr.shape(0));
}

// ============================================================================
// Check if numpy array shape is compatible with span<Matrix>
// ============================================================================

template <typename MatrixT>
bool is_span_of(numpy_array<typename std::remove_const_t<MatrixT>::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    constexpr int Rows = std::remove_const_t<MatrixT>::RowsAtCompileTime;
    constexpr int Cols = std::remove_const_t<MatrixT>::ColsAtCompileTime;

    if(arr.ndim() == 2)
    {
        if constexpr(Rows == 1 || Cols == 1)
        {
            if((size_t)arr.shape(1) != (size_t)(Rows * Cols))
                return false;
        }
        else
        {
            return false;
        }
    }
    else if(arr.ndim() == 3)
    {
        if(!((size_t)arr.shape(1) == (size_t)Rows
             && (size_t)arr.shape(2) == (size_t)Cols))
            return false;
    }
    else
    {
        return false;
    }

    return true;
}

// ============================================================================
// Copy Eigen matrix to a new numpy array (owning copy)
// ============================================================================

template <typename T, int M, int N, int Options>
py::object as_numpy(const Matrix<T, M, N, Options>& mat)
    requires(M > 0 && N > 0)
{
    using MatrixType = Eigen::Matrix<T, M, N, Options>;

    int64_t outer = MatrixType::OuterStrideAtCompileTime;
    int64_t inner = MatrixType::InnerStrideAtCompileTime;

    constexpr bool rowMajor = MatrixType::Flags & Eigen::RowMajorBit;
    if(!rowMajor)
        std::swap(outer, inner);

    size_t  shape[2]   = {(size_t)M, (size_t)N};
    int64_t strides[2] = {outer * (int64_t)sizeof(T), inner * (int64_t)sizeof(T)};

    // Create a temporary ndarray that wraps the matrix data
    py::ndarray<py::numpy, T> nd(
        (void*)mat.data(), 2, shape, py::handle(), strides);

    // Cast to Python; since owner is empty, nanobind will copy the data
    py::object view = py::cast(nd, py::rv_policy::none);

    // Force a copy to ensure the returned array owns its data
    auto numpy = py::module_::import_("numpy");
    return numpy.attr("array")(view);
}

template <typename T, int M, int N, int Options>
py::object as_numpy(Matrix<T, M, N, Options>& mat)
    requires(M > 0 && N > 0)
{
    return as_numpy(std::as_const(mat));
}

// Helper: map C++ types to numpy dtype strings
template <typename T>
constexpr const char* numpy_dtype_str()
{
    if constexpr(std::is_same_v<T, float>)
        return "float32";
    else if constexpr(std::is_same_v<T, double>)
        return "float64";
    else if constexpr(std::is_same_v<T, int32_t>)
        return "int32";
    else if constexpr(std::is_same_v<T, int64_t>)
        return "int64";
    else if constexpr(std::is_same_v<T, uint32_t>)
        return "uint32";
    else if constexpr(std::is_same_v<T, uint64_t>)
        return "uint64";
    else
        static_assert(sizeof(T) == 0, "Unsupported numpy dtype");
}

// ============================================================================
// Helper: create empty numpy array with given shape and dtype
// ============================================================================

template <typename T>
py::object make_numpy_empty(std::initializer_list<size_t> shape_init)
{
    auto    numpy = py::module_::import_("numpy");
    py::list shape_list;
    for(auto s : shape_init)
        shape_list.append((int64_t)s);
    return numpy.attr("empty")(shape_list, py::arg("dtype") = numpy_dtype_str<T>());
}

// ============================================================================
// Convert numpy array to Eigen matrix (copy)
// ============================================================================

template <typename MatrixT>
MatrixT to_matrix(numpy_array<typename MatrixT::Scalar> arr)
    requires requires(MatrixT) {
        MatrixT::RowsAtCompileTime > 0;
        MatrixT::ColsAtCompileTime > 0;
    }
{
    using Scalar       = typename MatrixT::Scalar;
    constexpr int Rows = MatrixT::RowsAtCompileTime;
    constexpr int Cols = MatrixT::ColsAtCompileTime;

    MatrixT m;

    if(arr.ndim() == 1)
    {
        if(Rows == 1 || Cols == 1)
        {
            PYUIPC_ASSERT(arr.shape(0) == (size_t)(Rows * Cols),
                          "Shape mismatch, ask for shape=({}), yours={}",
                          Rows * Cols,
                          arr.shape(0));
        }
        else
        {
            throw PyException(PYUIPC_MSG("array must be 2D, yours={}", arr.ndim()));
        }

        auto        count = std::max(Rows, Cols);
        const auto* data  = (const Scalar*)arr.data();

        for(int i = 0; i < count; i++)
            m(i) = data[i];
    }
    else if(arr.ndim() == 2)
    {
        PYUIPC_ASSERT(arr.shape(0) == (size_t)Rows && arr.shape(1) == (size_t)Cols,
                      "Shape mismatch, ask for shape=({},{}), yours=({},{})",
                      Rows,
                      Cols,
                      arr.shape(0),
                      arr.shape(1));

        for(int i = 0; i < Rows; i++)
            for(int j = 0; j < Cols; j++)
                m(i, j) = arr(i, j);
    }
    else
    {
        throw PyException(PYUIPC_MSG("array must be 1D or 2D, yours={}", arr.ndim()));
    }

    return m;
}
}  // namespace pyuipc
