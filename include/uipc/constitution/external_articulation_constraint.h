#pragma once
#pragma once
#include <uipc/common/type_define.h>
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API ExternalArticulationConstraint final : public IConstitution
{
    using Base = IConstitution;

  public:
    ExternalArticulationConstraint(const Json& config = default_config()) noexcept;

    ~ExternalArticulationConstraint();

    [[nodiscard]] geometry::Geometry create_geometry(span<S<const geometry::GeometrySlot>> joint_geos) const;

    [[nodiscard]] geometry::Geometry create_geometry(span<S<const geometry::GeometrySlot>> joint_geos,
                                                     span<IndexT> indices) const;

    static Json default_config();

  protected:
    virtual U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution