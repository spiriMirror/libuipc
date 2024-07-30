#include <pyuipc/pyuipc.h>
#include <pybind11/numpy.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <pyuipc/as_numpy.h>
#include <pybind11/eigen.h>
namespace pyuipc
{
// Pure C++ code
class SmartObjectA
{
  public:
    SmartObjectA() {}
    ~SmartObjectA() {}

    span<const double> view() { return span<const double>(my_vector); }

    std::vector<double> my_vector = {1, 2, 3, 4, 5};

    std::string_view name() { return "SmartObjectA"; }
};

template <typename T, size_t M, size_t N, int Options>
py::array_t<T, py::array::c_style> as_numpy(span<Eigen::Matrix<T, M, N, Options>> v)
    requires(M > 0 && N > 0)
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
                     stride_2 * (py::ssize_t)sizeof(double),
                     stride_3 * (py::ssize_t)sizeof(double)};
    info.format   = py::format_descriptor<double>::format();
    info.itemsize = sizeof(double);
    info.ptr      = (void*)v.data();
    info.readonly = false;

    return py::array_t<Matrix>(info);
}

template <typename T, size_t M, size_t N, int Options>
py::array_t<T, py::array::c_style> as_numpy(span<const Eigen::Matrix<T, M, N, Options>> v)
    requires(M > 0 && N > 0)
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
                     stride_2 * (py::ssize_t)sizeof(double),
                     stride_3 * (py::ssize_t)sizeof(double)};
    info.format   = py::format_descriptor<double>::format();
    info.itemsize = sizeof(double);
    info.ptr      = (void*)v.data();
    info.readonly = true;

    return py::array_t<const Matrix>(info);
}


class SmartObjectB
{
  public:
    SmartObjectB()
    {
        Matrix3x3 m;
        m.row(0) << 1, 2, 3;
        m.row(1) << 4, 5, 6;
        m.row(2) << 7, 8, 9;
        my_vector.push_back(m);
    }
    ~SmartObjectB() {}

    span<const Matrix3x3> view() { return span<const Matrix3x3>(my_vector); }

    std::vector<Matrix3x3> my_vector;
};

auto view(SmartObjectA& obj)
{
    return span{obj.my_vector};
}

auto view(SmartObjectB& obj)
{
    return span{obj.my_vector};
}

std::shared_ptr<SmartObjectA> create_smart_object()
{
    printf("c++: create_smart_object\n");
    return std::make_shared<SmartObjectA>();
}

void receive_smart_object(SmartObjectA& obj)
{
    printf("c++: receive_smart_object=%p\n", &obj);
}

static Module M(
    [](py::module& m)
    {
        // bind Matrix3d


        py::class_<SmartObjectA, std::shared_ptr<SmartObjectA>> obj(
            m, PYBIND11_TOSTRING(SmartObjectA));
        obj.def(py::init<>());

        // use lambda to avoid ambiguity
        obj.def("view", [](SmartObjectA& self) { return as_numpy(self.view()); });
        obj.def("name", &SmartObjectA::name);

        m.def("view", [](SmartObjectA& self) { return as_numpy(view(self)); });

        m.def("create_smart_object", &create_smart_object);
        m.def("receive_smart_object", &receive_smart_object);

        py::class_<SmartObjectB, std::shared_ptr<SmartObjectB>> obj_a(
            m, PYBIND11_TOSTRING(SmartObjectB));
        obj_a.def(py::init<>());

        obj_a.def("view",
                  [](SmartObjectB& self) { return as_numpy(self.view()); });
        m.def("view", [](SmartObjectB& self) { return as_numpy(view(self)); });
    });
}  // namespace pyuipc
