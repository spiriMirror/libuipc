/*
 * NVIDIA contributions: one numeric-slot overload of uipc.view with a
 * class-table argument caster.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pyuipc/geometry/attribute_slot.h>
#include <uipc/geometry/attribute_slot.h>
#include <pybind11/stl.h>
#include <sstream>
#include <vector>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

namespace
{
    // uipc.view(slot) for the slot classes AttributeSlot<T> of numeric T.
    // pybind11 tries the overloads of a function one by one, so one overload
    // per class made the view of the k-th class cost k failed tries (about
    // 0.3 us each). One overload whose argument caster finds the class in a
    // table accepts the same objects in one try.
    using NumericView = py::array (*)(py::handle slot);

    struct NumericViewEntry
    {
        PyTypeObject* type;
        NumericView   view;
    };

    // In registration order; filled once at import.
    std::vector<NumericViewEntry> numeric_views;

    template <typename T>
    py::array numeric_view(py::handle slot)
    {
        return as_numpy(view(slot.cast<AttributeSlot<T>&>()), slot);
    }

    NumericView find_numeric_view(PyTypeObject* type)
    {
        for(const auto& entry : numeric_views)
            if(entry.type == type)
                return entry.view;
        // A subclass matched its base class's overload before as well.
        for(const auto& entry : numeric_views)
            if(PyType_IsSubtype(type, entry.type))
                return entry.view;
        return nullptr;
    }

    // The argument of the numeric overload of uipc.view(): a slot of a
    // numeric class (the Python object, kept as the array's base) and the
    // view of its class.
    struct NumericSlot
    {
        py::handle  self;
        NumericView view = nullptr;
    };

    // The value types of the numeric slot classes registered below. The
    // signature names their union, as the former overloads named each class.
    template <typename... T>
    struct NumericSlotClasses
    {
#if PYBIND11_VERSION_MAJOR >= 3
        static constexpr auto name =
            py::detail::union_concat(py::detail::const_name<AttributeSlot<T>>()...);
#else
        // pybind11 2.x has no union_concat: name the union as its
        // std::variant caster does
        static constexpr auto name =
            py::detail::const_name("Union[")
            + py::detail::concat(py::detail::const_name<AttributeSlot<T>>()...)
            + py::detail::const_name("]");
#endif
        static constexpr std::size_t count = sizeof...(T);
    };

    using NumericSlotTypes =
        NumericSlotClasses<Float, I32, I64, U32, U64, Vector2, Vector3, Vector4, Vector6, Vector9, Vector12, Vector2i, Vector3i, Vector4i, Matrix2x2, Matrix3x3, Matrix4x4, Matrix6x6, Matrix9x9, Matrix12x12>;
}  // namespace
}  // namespace pyuipc::geometry

namespace pybind11::detail
{
// Loads exactly the objects the former per-class overloads accepted; string
// slots and other objects go on to the next overload.
template <>
struct type_caster<pyuipc::geometry::NumericSlot>
{
    PYBIND11_TYPE_CASTER(pyuipc::geometry::NumericSlot,
                         pyuipc::geometry::NumericSlotTypes::name);

    bool load(handle src, bool)
    {
        value.self = src;
        value.view = pyuipc::geometry::find_numeric_view(Py_TYPE(src.ptr()));
        return value.view != nullptr;
    }
};
}  // namespace pybind11::detail

namespace pyuipc::geometry
{
template <typename T>
void def_attribute_slot(py::module& m, std::string name)
{
    auto class_AttributeSlotT =
        py::class_<AttributeSlot<T>, IAttributeSlot, S<AttributeSlot<T>>>(m, name.c_str());

    class_AttributeSlotT.def(
        "view",
        [](AttributeSlot<T>& self)
        { return as_numpy(self.view(), py::cast(self)); },
        R"(Get a read-only NumPy view. To modify this attribute, use uipc.view(slot).
Returns:
    numpy.ndarray: Array view of attribute data.)");

    // uipc.view(slot) of this class (registered below, after all classes)
    numeric_views.push_back(
        {reinterpret_cast<PyTypeObject*>(class_AttributeSlotT.ptr()), &numeric_view<T>});
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
                         {
                             std::ostringstream oss;
                             oss << "[";
                             for(size_t i = 0; i < v.size(); ++i)
                             {
                                 if(i > 0)
                                     oss << ",";
                                 oss << v[i];
                             }
                             oss << "]";
                             return oss.str();
                         });
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
        "view",
        [](AttributeSlot<std::string>& self) { return self.view(); },
        py::keep_alive<0, 1>(),
        R"(Get a view of the string attribute data.
Returns:
    StringSpan: View of string attribute data.)");

    //m.def("view", [](AttributeSlot<std::string>& self) { return view(self); });

    top_module().def(
        "view",
        [](AttributeSlot<std::string>& self) { return view(self); },
        py::keep_alive<0, 1>(),
        py::arg("slot"),
        R"(Get a view of a string attribute slot.
Args:
    slot: AttributeSlotString to view.
Returns:
    StringSpan: View of string attribute data.)");
}

#define DEF_ATTRIBUTE_SLOT(T) def_attribute_slot<T>(m, "AttributeSlot" #T)

PyAttributeSlot::PyAttributeSlot(py::module& m)
{
    auto class_IAttributeSlot = py::class_<IAttributeSlot, S<IAttributeSlot>>(
        m, "IAttributeSlot", R"(IAttributeSlot interface for attribute slots.)");
    class_IAttributeSlot
        .def("name",
             &IAttributeSlot::name,
             R"(Get the attribute name.
Returns:
    str: Attribute name.)")
        .def("type_name",
             &IAttributeSlot::type_name,
             R"(Get the attribute type name.
Returns:
    str: Type name.)")
        .def("allow_destroy",
             &IAttributeSlot::allow_destroy,
             R"(Check if the attribute can be destroyed.
Returns:
    bool: True if destroyable, False otherwise.)")
        .def("is_shared",
             &IAttributeSlot::is_shared,
             R"(Check if the attribute is shared.
Returns:
    bool: True if shared, False otherwise.)")
        .def("size",
             &IAttributeSlot::size,
             R"(Get the size of the attribute.
Returns:
    int: Number of elements.)")
        // view pure virtual
        .def(
            "view",
            [](IAttributeSlot& self) -> py::array { return py::none(); },
            R"(Get a view of the attribute data (virtual method, returns None for base class).
Returns:
    numpy.ndarray or None: Array view if available, None otherwise.)");


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

    // One overload for all numeric slot classes, tried before the string
    // overload (and the ParameterCollection one, registered later): views of
    // numeric slots are the frequent ones.
    PYUIPC_ASSERT(numeric_views.size() == NumericSlotTypes::count,
                  "NumericSlotTypes must list the {} numeric slot classes, it lists {}",
                  numeric_views.size(),
                  NumericSlotTypes::count);
    top_module().def(
        "view",
        [](NumericSlot slot) { return slot.view(slot.self); },
        py::arg("slot"),
        R"(Get a writable NumPy view, detaching shared attribute storage if needed.
Args:
    slot: AttributeSlot to view.
Returns:
    numpy.ndarray: Array view of attribute data.)",
        py::prepend());
}
}  // namespace pyuipc::geometry
