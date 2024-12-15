#include <pyuipc/common/uipc_type.h>
#include <uipc/common/type_define.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc
{
template <typename T>
class Type
{
  public:
    using type = T;
};

template <typename T>
class ValueType
{
  public:
    using type = T;
};

// dummy class for type
class I32;
class I64;
class U32;
class U64;
class Float;
class IndexT;

#define DEF_VALUE_TYPE(Type, VT)                                               \
    template <>                                                                \
    class ValueType<Type>                                                      \
    {                                                                          \
      public:                                                                  \
        using type = VT;                                                       \
    }

DEF_VALUE_TYPE(I32, uipc::I32);
DEF_VALUE_TYPE(I64, uipc::I64);
DEF_VALUE_TYPE(U32, uipc::U32);
DEF_VALUE_TYPE(U64, uipc::U64);
DEF_VALUE_TYPE(Float, uipc::Float);
DEF_VALUE_TYPE(IndexT, uipc::IndexT);

template <typename T>
void def_scalar(py::module& m, const char* name)
{
    using Ty = Type<T>;
    using VT = typename ValueType<T>::type;

    auto class_Scalar = py::class_<Ty>(m, name);

    class_Scalar.def_static("Zero", []() { return VT{0}; });
    class_Scalar.def_static("One", []() { return VT{1}; });
    class_Scalar.def_static("Value", [](VT value) { return value; });
    class_Scalar.def_static("size_bytes", []() { return sizeof(VT); });
}

template <typename T, int M, int N>
void def_matrix(py::module& m, Eigen::Matrix<T, M, N>, const char* name)
{
    using Ty = Type<Eigen::Matrix<T, M, N>>;

    // derived from numpy array

    auto class_Matrix = py::class_<Ty>(m, name);

    class_Matrix.def_static("shape", []() { return std::make_tuple(M, N); });

    // return numpy array zeros
    class_Matrix.def_static("Zero",
                            []()
                            {
                                Eigen::Matrix<T, M, N> mat =
                                    Eigen::Matrix<T, M, N>::Zero();
                                return as_numpy(mat);
                            });

    class_Matrix.def_static("Ones",
                            []()
                            {
                                Eigen::Matrix<T, M, N> mat =
                                    Eigen::Matrix<T, M, N>::Ones();
                                return as_numpy(mat);
                            });

    class_Matrix.def_static("Identity",
                            []()
                            {
                                Eigen::Matrix<T, M, N> mat =
                                    Eigen::Matrix<T, M, N>::Identity();
                                return as_numpy(mat);
                            });

    class_Matrix.def_static("Random",
                            []()
                            {
                                Eigen::Matrix<T, M, N> mat =
                                    Eigen::Matrix<T, M, N>::Random();
                                return as_numpy(mat);
                            });

    class_Matrix.def_static("Values",
                            [](py::array_t<T> value) {
                                return as_numpy(to_matrix<Eigen::Matrix<T, M, N>>(value));
                            });

    if constexpr(N == 1)
    {
        class_Matrix.def_static("LinSpaced",
                                [](T start, T end, int n)
                                {
                                    Eigen::Matrix<T, M, N> mat =
                                        Eigen::Matrix<T, M, N>::LinSpaced(start, end, n);
                                    return as_numpy(mat);
                                });

        class_Matrix.def_static("LinSpaced",
                                [](T start, T end)
                                {
                                    Eigen::Matrix<T, M, N> mat =
                                        Eigen::Matrix<T, M, N>::LinSpaced(start, end);
                                    return as_numpy(mat);
                                });

        class_Matrix.def_static("LinSpaced",
                                [](T start, T end, T step)
                                {
                                    Eigen::Matrix<T, M, N> mat =
                                        Eigen::Matrix<T, M, N>::LinSpaced(start, end, step);
                                    return as_numpy(mat);
                                });

        class_Matrix.def_static(
            "Unit",
            [](int i)
            {
                if(i < 0 || i >= M)
                {
                    throw std::runtime_error(
                        PYUIPC_MSG("Index out of range, size={}, yours={}", M, i));
                }

                Eigen::Matrix<T, M, N> mat = Eigen::Matrix<T, M, N>::Unit(i);
                return as_numpy(mat);
            });

        if constexpr(M >= 1)
        {
            class_Matrix.def_static("UnitX",
                                    []()
                                    {
                                        Eigen::Matrix<T, M, N> mat =
                                            Eigen::Matrix<T, M, N>::UnitX();
                                        return as_numpy(mat);
                                    });
        }

        if constexpr(M >= 2)
        {
            class_Matrix.def_static("UnitY",
                                    []()
                                    {
                                        Eigen::Matrix<T, M, N> mat =
                                            Eigen::Matrix<T, M, N>::UnitY();
                                        return as_numpy(mat);
                                    });
        }

        if constexpr(M >= 3)
        {
            class_Matrix.def_static("UnitZ",
                                    []()
                                    {
                                        Eigen::Matrix<T, M, N> mat =
                                            Eigen::Matrix<T, M, N>::UnitZ();
                                        return as_numpy(mat);
                                    });
        }

        if constexpr(M >= 4)
        {
            class_Matrix.def_static("UnitW",
                                    []()
                                    {
                                        Eigen::Matrix<T, M, N> mat =
                                            Eigen::Matrix<T, M, N>::UnitW();
                                        return as_numpy(mat);
                                    });
        }
    }
}

#define DEF_SCALAR(Type) def_scalar<Type>(m, #Type)
#define DEF_MATRIX(Type) def_matrix(m, Type{}, #Type)

PyUIPCType::PyUIPCType(py::module& m)
{
    DEF_SCALAR(Float);
    DEF_SCALAR(IndexT);
    DEF_SCALAR(SizeT);
    DEF_SCALAR(I32);
    DEF_SCALAR(I64);
    DEF_SCALAR(U32);
    DEF_SCALAR(U64);


    DEF_MATRIX(Vector2);
    DEF_MATRIX(Vector3);
    DEF_MATRIX(Vector4);
    DEF_MATRIX(Vector6);
    DEF_MATRIX(Vector9);
    DEF_MATRIX(Vector12);

    DEF_MATRIX(Matrix2x2);
    DEF_MATRIX(Matrix3x3);
    DEF_MATRIX(Matrix4x4);
    DEF_MATRIX(Matrix6x6);
    DEF_MATRIX(Matrix9x9);
    DEF_MATRIX(Matrix12x12);
}
}  // namespace pyuipc
