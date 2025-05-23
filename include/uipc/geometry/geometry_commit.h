#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/attribute_collection_commit.h>

namespace uipc::geometry
{
class UIPC_CORE_API GeometryCommit
{
    friend class Geometry;
    friend class GeometryCollection;
    friend class GeometryFactory;
    friend class GeometryAtlasCommit;

    friend UIPC_CORE_API GeometryCommit operator-(const Geometry& dst, const Geometry& src);
    friend UIPC_CORE_API Geometry& operator+=(Geometry& base, const GeometryCommit& inc);

  public:
    GeometryCommit() = default;
    GeometryCommit(const GeometryCommit&);
    GeometryCommit& operator=(const GeometryCommit&) = delete;

    explicit GeometryCommit(const Geometry& dst);
    bool is_valid() const noexcept { return m_is_valid; }
    bool is_new() const noexcept { return m_new_geometry != nullptr; }
    bool is_modification() const noexcept
    {
        return !m_new_geometry && m_attribute_collections.size() > 0;
    }
    S<Geometry>        new_geometry() const noexcept { return m_new_geometry; }
    const std::string& type() const noexcept { return m_type; }
    const unordered_map<std::string, S<AttributeCollectionCommit>>& attribute_collections() const noexcept
    {
        return m_attribute_collections;
    }

  private:
    GeometryCommit(const Geometry& dst, const Geometry& src);

    bool        m_is_valid = true;
    S<Geometry> m_new_geometry;  // the new geometry

    std::string m_type;
    unordered_map<std::string, S<AttributeCollectionCommit>> m_attribute_collections;
};

UIPC_CORE_API GeometryCommit operator-(const Geometry& dst, const Geometry& src);
UIPC_CORE_API Geometry& operator+=(Geometry& base, const GeometryCommit& inc);
}  // namespace uipc::geometry