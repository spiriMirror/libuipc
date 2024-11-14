#include <pyuipc/geometry/attribute_slot.h>
#include <uipc/geometry/attribute_slot.h>
#include <pyuipc/as_numpy.h>
#include <pybind11/stl.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;

template <typename T>
void def_attribute_slot(py::module& m, std::string name)
{
    auto class_AttributeSlotT =
        py::class_<AttributeSlot<T>, IAttributeSlot, S<AttributeSlot<T>>>(m, name.c_str());

    class_AttributeSlotT.def("view",
                             [](AttributeSlot<T>& self)
                             { return as_numpy(self.view(), py::cast(self)); });

    top_module().def("view",
                     [](AttributeSlot<T>& self)
                     { return as_numpy(view(self), py::cast(self)); });

    //m.def("view",
    //      [](AttributeSlot<T>& self)
    //      { return as_numpy(view(self), py::cast(self)); });
}

template <bool IsConst>
void def_class_StringSpan(py::module& m)
{
    using T    = std::conditional_t<IsConst, const std::string, std::string>;
    using LRef = std::add_lvalue_reference_t<T>;

    constexpr std::string_view name = IsConst ? "CStringSpan" : "StringSpan";

    auto class_StringSpan = py::class_<span<T>>(m, name.data());

    class_StringSpan.def(py::init<>())
        .def("__len__", [](span<T>& v) { return v.size(); })
        .def(
            "__iter__",
            [](span<T>& v) { return py::make_iterator(v.begin(), v.end()); },
            py::keep_alive<0, 1>())
        .def("__getitem__",
             [](span<T>& v, size_t i) -> LRef
             {
                 if(i >= v.size())
                 {
                     throw py::index_error();
                 }
                 return v[i];
             });

    if constexpr(!IsConst)
    {
        class_StringSpan.def("__setitem__",
                             [](span<T>& v, size_t i, LRef value)
                             {
                                 if(i >= v.size())
                                 {
                                     throw py::index_error();
                                 }
                                 v[i] = value;
                             });
    }

    class_StringSpan.def("__repr__",
                         [](span<T>& v)
                         { return fmt::format("[{}]", fmt::join(v, ",")); });
}


void def_attribute_slot_string(py::module& m)
{
    // const
    def_class_StringSpan<true>(m);
    // non-const
    def_class_StringSpan<false>(m);

    auto class_AttributeSlotString =
        py::class_<AttributeSlot<std::string>, IAttributeSlot, S<AttributeSlot<std::string>>>(
            m, "AttributeSlotString");
    class_AttributeSlotString.def(
        "view", [](AttributeSlot<std::string>& self) { return self.view(); });

    //m.def("view", [](AttributeSlot<std::string>& self) { return view(self); });

    top_module().def("view",
                     [](AttributeSlot<std::string>& self) { return view(self); });
}

#define DEF_ATTRIBUTE_SLOT(T) def_attribute_slot<T>(m, "AttributeSlot" #T)

PyAttributeSlot::PyAttributeSlot(py::module& m)
{
    auto class_IAttributeSlot =
        py::class_<IAttributeSlot, S<IAttributeSlot>>(m, "IAttributeSlot");
    class_IAttributeSlot.def("name", &IAttributeSlot::name)
        .def("type_name", &IAttributeSlot::type_name)
        .def("allow_destroy", &IAttributeSlot::allow_destroy)
        .def("is_shared", &IAttributeSlot::is_shared)
        .def("size", &IAttributeSlot::size)
        // view pure virtual
        .def("view", [](IAttributeSlot& self) -> py::array { return py::none(); });


    // String span
    def_attribute_slot_string(m);

    // Basic types
    DEF_ATTRIBUTE_SLOT(Float);
    DEF_ATTRIBUTE_SLOT(I32);
    DEF_ATTRIBUTE_SLOT(I64);
    DEF_ATTRIBUTE_SLOT(U32);
    DEF_ATTRIBUTE_SLOT(U64);

    // Vector types
    DEF_ATTRIBUTE_SLOT(Vector2);
    DEF_ATTRIBUTE_SLOT(Vector3);
    DEF_ATTRIBUTE_SLOT(Vector4);
    DEF_ATTRIBUTE_SLOT(Vector6);
    DEF_ATTRIBUTE_SLOT(Vector9);
    DEF_ATTRIBUTE_SLOT(Vector12);

    DEF_ATTRIBUTE_SLOT(Vector2i);
    DEF_ATTRIBUTE_SLOT(Vector3i);
    DEF_ATTRIBUTE_SLOT(Vector4i);

    // Matrix types
    DEF_ATTRIBUTE_SLOT(Matrix2x2);
    DEF_ATTRIBUTE_SLOT(Matrix3x3);
    DEF_ATTRIBUTE_SLOT(Matrix4x4);
    DEF_ATTRIBUTE_SLOT(Matrix6x6);
    DEF_ATTRIBUTE_SLOT(Matrix9x9);
    DEF_ATTRIBUTE_SLOT(Matrix12x12);
}
}  // namespace pyuipc::geometry
