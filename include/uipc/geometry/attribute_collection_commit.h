#pragma once
#include <uipc/geometry/attribute_collection.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeCollectionCommit
{
    friend class GeometryAtlasCommit;
    friend class GeometryCommit;
    friend class AttributeCollection;
    friend class AttributeCollectionFactory;
    friend class GeometryFactory;
    friend UIPC_CORE_API AttributeCollectionCommit operator-(const AttributeCollection& dst,
                                                             const AttributeCollection& src);
    friend UIPC_CORE_API AttributeCollection& operator+=(AttributeCollection& dst,
                                                         const AttributeCollectionCommit& inc);

  public:
    AttributeCollectionCommit() = default;
    explicit AttributeCollectionCommit(const AttributeCollection& dst);
    AttributeCollectionCommit(const AttributeCollectionCommit& other) = default;
    AttributeCollectionCommit& operator=(const AttributeCollectionCommit& other) = delete;

    const AttributeCollection& attribute_collection() const noexcept
    {
        return m_attribute_collection;
    }

    span<const std::string> removed_names() const noexcept
    {
        return span<const std::string>(m_removed_names);
    }

  private:
    AttributeCollectionCommit(const AttributeCollection& dst, const AttributeCollection& src);
    AttributeCollection m_attribute_collection;
    vector<std::string> m_removed_names;
};

UIPC_CORE_API AttributeCollectionCommit operator-(const AttributeCollection& dst,
                                                  const AttributeCollection& src);

UIPC_CORE_API AttributeCollection& operator+=(AttributeCollection& dst,
                                              const AttributeCollectionCommit& inc);
}  // namespace uipc::geometry