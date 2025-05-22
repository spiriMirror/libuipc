#pragma once
#include <uipc/geometry/attribute.h>
#include <uipc/geometry/attribute_slot.h>
#include <uipc/common/unordered_map.h>

namespace uipc::geometry
{
class UIPC_CORE_API SerialSharedAttributeContext
{
  public:
    SerialSharedAttributeContext() = default;

    [[nodiscard]] IndexT index_of(IAttribute* attr) const;

    [[nodiscard]] IAttribute* attribute_of(IndexT index) const;

  private:
    friend class GeometryAtlas;
    friend class GeometryAtlasCommit;

    unordered_map<IAttribute*, IndexT> m_attr_to_index;
    vector<IAttribute*>                m_index_to_attr;
    void                               clear();
};

class UIPC_CORE_API DeserialSharedAttributeContext
{
  public:
    DeserialSharedAttributeContext() = default;

    [[nodiscard]] S<IAttributeSlot> attribute_slot_of(IndexT index) const;

  private:
    friend class GeometryAtlas;
    friend class GeometryAtlasCommit;

    vector<S<IAttributeSlot>> m_attribute_slots;
    void                      clear();
};
}  // namespace uipc::geometry
