#include <pyuipc/geometry/attribute_collection.h>
#include <uipc/geometry/attribute_collection.h>
#include <pyuipc/as_numpy.h>
#include <uipc/common/type_traits.h>
#include <pybind11/numpy.h>
#include <boost/core/demangle.hpp>
namespace pyuipc::geometry
{
using namespace uipc::geometry;

template <typename T>
void disable_create_scalar_like_this(py::class_<AttributeCollection>& class_AttributeCollection)
    requires(!is_matrix_v<T>)
{
    class_AttributeCollection.def(
        "create",
        [](AttributeCollection& self, std::string_view name, T& value) -> S<IAttributeSlot>
        {
            auto type_name = boost::core::demangle(typeid(T).name());
            fmt::print("scalar type check: <{}>\n", type_name);
            throw std::runtime_error(
                R"(-- Misuage of `create` method when create scalar attribute --
Don't use: `create('NAME', V)`, which may cause ambiguous. Instead, try: `create('NAME', numpy.array(V, dtype=Type))`.)");
        });
}

void def_create(py::class_<AttributeCollection>& class_AttributeCollection)
{
    class_AttributeCollection
        .def("create",
             [](AttributeCollection& self, std::string_view name, py::array_t<Float> arr) -> S<IAttributeSlot>
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
                         throw std::runtime_error("Unsupported vector size");
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
                         throw std::runtime_error("Unsupported matrix size");
                     }
                 }
                 else
                 {
                     throw std::runtime_error("Unsupported shape of float64");
                 }
             })
        .def("create",
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
                         throw std::runtime_error("Unsupported vector size");
                     }
                 }
                 else
                 {
                     throw std::runtime_error("Unsupported shape of int");
                 }
             })
        .def("create",
             [](AttributeCollection& self, std::string_view name, py::array_t<U64> arr) -> S<IAttributeSlot>
             {
                 bool is_scalar = arr.ndim() == 0;
                 if(is_scalar)
                 {
                     return self.create(name, *arr.data());
                 }

                 throw std::runtime_error("Unsupported shape of uint64");
             })
        .def("create",
             [](AttributeCollection& self, std::string_view name, py::array_t<I64> arr) -> S<IAttributeSlot>
             {
                 bool is_scalar = arr.ndim() == 0;
                 if(is_scalar)
                 {
                     return self.create(name, *arr.data());
                 }

                 throw std::runtime_error("Unsupported shape of int64");
             });
}

PyAttributeCollection::PyAttributeCollection(py::module& m)
{
    auto class_AttributeCollection =
        py::class_<AttributeCollection>(m, "AttributeCollection");

    def_create(class_AttributeCollection);

    disable_create_scalar_like_this<U64>(class_AttributeCollection);
    disable_create_scalar_like_this<I64>(class_AttributeCollection);
    disable_create_scalar_like_this<Float>(class_AttributeCollection);

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
