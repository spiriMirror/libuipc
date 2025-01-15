#pragma once
#include <pyuipc/pyuipc.h>
#include <uipc/geometry/attribute_slot.h>
#include <uipc/geometry/attribute_collection.h>

namespace pyuipc::geometry
{
class AttributeCreator
{
  public:
    static S<uipc::geometry::IAttributeSlot> create(uipc::geometry ::AttributeCollection& a,
                                                    std::string_view name,
                                                    py::object       object)
    {
        auto pyobj = py::cast(a, py::return_value_policy::reference);

        // call the create method of the member object
        return py::cast<S<uipc::geometry::IAttributeSlot>>(
            pyobj.attr("create").operator()(py::cast(name), object));
    }
};
}  // namespace pyuipc::geometry