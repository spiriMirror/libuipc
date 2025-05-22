#include <uipc/geometry/shared_attribute_context.h>

namespace uipc::geometry
{
IndexT SerialSharedAttributeContext::index_of(IAttribute* attr) const
{
    auto it = m_attr_to_index.find(attr);
    if(it != m_attr_to_index.end())
        return it->second;
    return -1;
}

IAttribute* SerialSharedAttributeContext::attribute_of(IndexT index) const
{
    if(index < 0 || index >= m_index_to_attr.size())
        return nullptr;
    return m_index_to_attr[index];
}

void SerialSharedAttributeContext::clear()
{
    m_attr_to_index.clear();
    m_index_to_attr.clear();
}

S<IAttributeSlot> DeserialSharedAttributeContext::attribute_slot_of(IndexT index) const
{
    if(index < 0 || index >= m_attribute_slots.size())
        return nullptr;
    return m_attribute_slots[index];
}

void DeserialSharedAttributeContext::clear()
{
    m_attribute_slots.clear();
}
}  // namespace uipc::geometry
