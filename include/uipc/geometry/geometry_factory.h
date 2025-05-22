#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/geometry_commit.h>
namespace uipc::geometry
{
class UIPC_CORE_API GeometryFactory
{
    class Impl;

  public:
    GeometryFactory();
    ~GeometryFactory();

    [[nodiscard]] vector<S<Geometry>> from_json(const Json& j,
                                                span<S<IAttributeSlot>> attributes);

    [[nodiscard]] Json to_json(span<Geometry*> geos,
                               unordered_map<IAttribute*, IndexT> attr_to_index);

    [[nodiscard]] Json to_json(Geometry& geos, unordered_map<IAttribute*, IndexT> attr_to_index);

    [[nodiscard]] Json commit_to_json(const GeometryCommit& gc,
                                      unordered_map<IAttribute*, IndexT> attr_to_index);

    [[nodiscard]] S<GeometryCommit> commit_from_json(const Json& j,
                                                     span<S<IAttributeSlot>> attributes);

    [[nodiscard]] S<GeometrySlot> create_slot(IndexT id, const Geometry& geometry);

    /* [[nodiscard]] S<Geometry> create_geometry(std::string_view type);*/

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
