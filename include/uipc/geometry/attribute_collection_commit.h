#pragma once
#include <uipc/geometry/attribute_collection.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeCollectionCommit
{
    friend class GeometryCommit;
    friend class AttributeCollectionFactory;
    friend UIPC_CORE_API AttributeCollectionCommit operator-(const AttributeCollection& dst,
                                                             const AttributeCollection& src);

  public:
    explicit AttributeCollectionCommit(const AttributeCollection& dst);

  private:
    AttributeCollectionCommit() = default;
    AttributeCollectionCommit(const AttributeCollection& dst, const AttributeCollection& src);


    AttributeCollection m_inc;
    vector<std::string> m_removed_names;
};

UIPC_CORE_API AttributeCollectionCommit operator-(const AttributeCollection& dst,
                                                  const AttributeCollection& src);

UIPC_CORE_API AttributeCollection& operator+=(AttributeCollection& dst,
                                              const AttributeCollectionCommit& inc);
}  // namespace uipc::geometry