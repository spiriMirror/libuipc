#pragma once
#include <uipc/common/macro.h>
#include <uipc/geometry/attribute_slot.h>
#include <uipc/geometry/geometry_slot.h>

namespace uipc::diff_sim
{
class UIPC_CORE_API ParameterBundle
{
    friend class ParameterCollection;

    ParameterBundle(S<geometry::GeometrySlot>          geo_slot,
                    S<geometry::IAttributeSlot>        parm_attr,
                    S<geometry::AttributeSlot<IndexT>> mapping);
    ~ParameterBundle();

  public:
    class Interface;

  private:
    U<Interface> m_impl;
};
}  // namespace uipc::diff_sim
