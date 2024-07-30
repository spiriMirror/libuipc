#include <pyuipc/geometry/attribute_slot.h>
#include <uipc/geometry/attribute_slot.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

template <typename T>
void def_attribute_slot(py::module& m, std::string name)
{
    auto class_AttributeSlotT =
        py::class_<AttributeSlot<T>, IAttributeSlot>(m, name.c_str());

    class_AttributeSlotT.def(
        "view", [](AttributeSlot<T>& self) { return as_numpy(self.view()); });

    m.def("view", [](AttributeSlot<T>& self) { return view(self); });
}

#define DEF_ATTRIBUTE_SLOT(T) def_attribute_slot<T>(m, "AttributeSlot" #T)

PyAttributeSlot::PyAttributeSlot(py::module& m)
{
    auto class_IAttributeSlot = py::class_<IAttributeSlot>(m, "IAttributeSlot");
    class_IAttributeSlot.def("name", &IAttributeSlot::name)
        .def("type_name", &IAttributeSlot::type_name)
        .def("allow_destroy", &IAttributeSlot::allow_destroy)
        .def("is_shared", &IAttributeSlot::is_shared)
        .def("size", &IAttributeSlot::size);

    // basic types
    DEF_ATTRIBUTE_SLOT(Float);
    DEF_ATTRIBUTE_SLOT(IndexT);
    DEF_ATTRIBUTE_SLOT(SizeT);

    // Vector types
    DEF_ATTRIBUTE_SLOT(Vector2);
    DEF_ATTRIBUTE_SLOT(Vector3);
    DEF_ATTRIBUTE_SLOT(Vector4);
    DEF_ATTRIBUTE_SLOT(Vector6);
    DEF_ATTRIBUTE_SLOT(Vector9);
    DEF_ATTRIBUTE_SLOT(Vector12);

    // Matrix types
    DEF_ATTRIBUTE_SLOT(Matrix2x2);
    DEF_ATTRIBUTE_SLOT(Matrix3x3);
    DEF_ATTRIBUTE_SLOT(Matrix4x4);
    DEF_ATTRIBUTE_SLOT(Matrix6x6);
    DEF_ATTRIBUTE_SLOT(Matrix9x9);
    DEF_ATTRIBUTE_SLOT(Matrix12x12);
}
}  // namespace pyuipc::geometry
