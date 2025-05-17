#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/attribute_collection_commit.h>

namespace uipc::geometry
{
class UIPC_CORE_API GeometryCommit
{
    friend class GeometryFactory;
    friend UIPC_CORE_API GeometryCommit operator-(const Geometry& dst, const Geometry& src);
    friend UIPC_CORE_API Geometry& operator+=(Geometry& base, const GeometryCommit& inc);

  public:
    GeometryCommit() = default;
    explicit GeometryCommit(const Geometry& dst);
    bool is_valid() const noexcept { return m_is_valid; }
    bool is_new() const noexcept { return m_new_geometry != nullptr; }
    bool is_modification() const noexcept
    {
        return !m_new_geometry && m_commits.size() > 0;
    }

  private:
    GeometryCommit(const Geometry& dst, const Geometry& src);

    bool        m_is_valid = true;
    S<Geometry> m_new_geometry;  // the new geometry

    std::string                       m_type;
    vector<std::string>               m_names;
    vector<AttributeCollectionCommit> m_commits;
};

UIPC_CORE_API GeometryCommit operator-(const Geometry& dst, const Geometry& src);
UIPC_CORE_API Geometry& operator+=(Geometry& base, const GeometryCommit& inc);
}  // namespace uipc::geometry