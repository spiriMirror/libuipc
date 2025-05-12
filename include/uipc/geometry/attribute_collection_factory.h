#pragma once
#include <uipc/geometry/attribute_collection.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeCollectionFactory
{
    class Impl;

  public:
    AttributeCollectionFactory();
    ~AttributeCollectionFactory();

    [[nodiscard]] S<AttributeCollection> from_json(const Json& j,
                                                   span<S<IAttributeSlot>> attributes);

    [[nodiscard]] Json to_json(const AttributeCollection* acs,
                               unordered_map<IAttribute*, IndexT> attr_to_index);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
