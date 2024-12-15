#include <pyuipc/geometry/attribute_collection.h>
#include <uipc/geometry/attribute_collection.h>
#include <pyuipc/as_numpy.h>
#include <uipc/common/type_traits.h>
#include <pybind11/numpy.h>
#include <boost/core/demangle.hpp>
namespace pyuipc::geometry
{
using namespace uipc::geometry;

void def_create(py::class_<AttributeCollection>& class_AttributeCollection)
{
    class_AttributeCollection
        // I32
        .def(
            "create",
            [](AttributeCollection& self, std::string_view name, I32 value) -> S<IAttributeSlot>
            { return self.create(name, value); },
            py::arg("name"),
            py::arg("value").noconvert())
        // I64
        .def(
            "create",
            [](AttributeCollection& self, std::string_view name, I64 arr) -> S<IAttributeSlot>
            { return self.create(name, arr); },
            py::arg("name"),
            py::arg("value").noconvert())
        // U64
        .def(
            "create",
            [](AttributeCollection& self, std::string_view name, U64 arr) -> S<IAttributeSlot>
            { return self.create(name, arr); },
            py::arg("name"),
            py::arg("value").noconvert())
        // Float
        .def(
            "create",
            [](AttributeCollection& self, std::string_view name, Float arr) -> S<IAttributeSlot>
            { return self.create(name, arr); },
            py::arg("name"),
            py::arg("value").noconvert())
        // Float Array
        .def(
            "create",
            [](AttributeCollection& self, std::string_view name, py::array_t<Float> arr) -> S<IAttributeSlot>
            {
                bool is_scalar =
                    arr.ndim() == 0 || (arr.ndim() == 1 && arr.shape(0) == 1)
                    || (arr.ndim() == 2 && arr.shape(0) == 1 && arr.shape(1) == 1);
                if(is_scalar)
                {
                    return self.create(name, *arr.data());
                }

                bool is_vector = arr.ndim() == 1
                                 || (arr.ndim() == 2 && arr.shape(1) == 1)
                                 || (arr.ndim() == 2 && arr.shape(0) == 1);

                if(is_vector)  // Vector Type
                {
                    if(arr.shape(0) == 2)
                    {
                        return self.create(name, to_matrix<Vector2>(arr));
                    }
                    else if(arr.shape(0) == 3)
                    {
                        return self.create(name, to_matrix<Vector3>(arr));
                    }
                    else if(arr.shape(0) == 4)
                    {
                        return self.create(name, to_matrix<Vector4>(arr));
                    }
                    else if(arr.shape(0) == 6)
                    {
                        return self.create(name, to_matrix<Vector6>(arr));
                    }
                    else if(arr.shape(0) == 9)
                    {
                        return self.create(name, to_matrix<Vector9>(arr));
                    }
                    else if(arr.shape(0) == 12)
                    {
                        return self.create(name, to_matrix<Vector12>(arr));
                    }
                    else
                    {
                        throw std::runtime_error(PYUIPC_MSG("Unsupported vector size"));
                    }
                }
                else if(arr.ndim() == 2)  // Matrix Type or Vector Type
                {

                    if(arr.shape(0) == 2 && arr.shape(1) == 2)
                    {
                        return self.create(name, to_matrix<Matrix2x2>(arr));
                    }
                    else if(arr.shape(0) == 3 && arr.shape(1) == 3)
                    {
                        return self.create(name, to_matrix<Matrix3x3>(arr));
                    }
                    else if(arr.shape(0) == 4 && arr.shape(1) == 4)
                    {
                        return self.create(name, to_matrix<Matrix4x4>(arr));
                    }
                    else if(arr.shape(0) == 6 && arr.shape(1) == 6)
                    {
                        return self.create(name, to_matrix<Matrix6x6>(arr));
                    }
                    else if(arr.shape(0) == 9 && arr.shape(1) == 9)
                    {
                        return self.create(name, to_matrix<Matrix9x9>(arr));
                    }
                    else if(arr.shape(0) == 12 && arr.shape(1) == 12)
                    {
                        return self.create(name, to_matrix<Matrix12x12>(arr));
                    }
                    else
                    {
                        throw std::runtime_error(PYUIPC_MSG("Unsupported matrix size"));
                    }
                }
                else
                {
                    throw std::runtime_error(PYUIPC_MSG("Unsupported shape of float64"));
                }
            },
            py::arg("name"),
            py::arg("value").noconvert())
        // Int Array
        .def(
            "create",
            [](AttributeCollection& self, std::string_view name, py::array_t<IndexT> arr) -> S<IAttributeSlot>
            {
                bool is_scalar = arr.ndim() == 0;
                if(is_scalar)
                {
                    return self.create(name, *arr.data());
                }

                bool is_vector =
                    arr.ndim() == 1 || (arr.ndim() == 2 && arr.shape(1) == 1);

                if(is_vector)  // Vector Type
                {
                    if(arr.shape(0) == 2)
                    {
                        return self.create(name, to_matrix<Vector2i>(arr));
                    }
                    else if(arr.shape(0) == 3)
                    {
                        return self.create(name, to_matrix<Vector3i>(arr));
                    }
                    else if(arr.shape(0) == 4)
                    {
                        return self.create(name, to_matrix<Vector4i>(arr));
                    }
                    else
                    {
                        throw std::runtime_error(PYUIPC_MSG("Unsupported vector size"));
                    }
                }
                else
                {
                    throw std::runtime_error(PYUIPC_MSG("Unsupported shape of int"));
                }
            },
            py::arg("name"),
            py::arg("value").noconvert())
        // String
        .def("create",
             [](AttributeCollection& self, std::string_view name, std::string_view str)
             { return self.create<std::string>(name, std::string{str}); });
}

PyAttributeCollection::PyAttributeCollection(py::module& m)
{
    auto class_AttributeCollection =
        py::class_<AttributeCollection>(m, "AttributeCollection");

    def_create(class_AttributeCollection);

    class_AttributeCollection.def(
        "share",
        [](AttributeCollection& self, std::string_view name, IAttributeSlot& slot)
        { self.share(name, slot); });

    class_AttributeCollection.def("destroy", &AttributeCollection::destroy);

    class_AttributeCollection.def("find",
                                  [](AttributeCollection& self,
                                     std::string_view name) -> S<IAttributeSlot>
                                  { return self.find(name); });

    class_AttributeCollection.def("resize", &AttributeCollection::resize);

    class_AttributeCollection.def("clear", &AttributeCollection::clear);

    class_AttributeCollection.def("size", &AttributeCollection::size);

    class_AttributeCollection.def("reserve", &AttributeCollection::reserve);

    class_AttributeCollection.def("attribute_count", &AttributeCollection::attribute_count);

    class_AttributeCollection.def("reorder",
                                  [](AttributeCollection& self, py::array_t<SizeT> arr)
                                  { self.reorder(as_span<SizeT>(arr)); });
}
}  // namespace pyuipc::geometry
