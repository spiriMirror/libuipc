#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/geometry_commit.h>
#include <uipc/geometry/shared_attribute_context.h>

namespace uipc::geometry
{
class UIPC_CORE_API GeometryFactory
{
    class Impl;

  public:
    GeometryFactory();
    ~GeometryFactory();

    [[nodiscard]] vector<S<Geometry>> from_json(const Json& j,
                                                DeserialSharedAttributeContext& ctx);

    [[nodiscard]] Json to_json(span<Geometry*> geos, SerialSharedAttributeContext& ctx);

    [[nodiscard]] Json to_json(Geometry& geos, SerialSharedAttributeContext& ctx);

    [[nodiscard]] Json commit_to_json(const GeometryCommit&         gc,
                                      SerialSharedAttributeContext& ctx);

    [[nodiscard]] S<GeometryCommit> commit_from_json(const Json& j,
                                                     DeserialSharedAttributeContext& ctx);

    [[nodiscard]] S<GeometrySlot> create_slot(IndexT id, const Geometry& geometry);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
