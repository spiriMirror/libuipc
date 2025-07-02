#pragma once
#include <uipc/constitution/inter_primitive_constitution.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API SoftVertexStitch final : public InterPrimitiveConstitution
{
    using Base = InterPrimitiveConstitution;

  public:
    using SlotTuple =
        std::tuple<S<geometry::SimplicialComplexSlot>, S<geometry::SimplicialComplexSlot>>;

    SoftVertexStitch(const Json& config = default_config());

    /**
     * @brief Create soft vertex stitch constraints between two Geometries
     * 
     * @param aim_geo_slots The slots of the two geometries to be stitched
     * @param stitched_vert_ids Each element is a pair of vertex ids, 
     *        the first one is from the first geometry, the second one is from the second geometry
     * @param kappa The stiffness of the stitch constraint
     */
    geometry::Geometry create_geometry(const SlotTuple&     aim_geo_slots,
                                       span<const Vector2i> stitched_vert_ids,
                                       Float                kappa = 1e6) const;

    static Json default_config();

  private:
    U64  get_uid() const noexcept override;
    Json m_config;
};
}  // namespace uipc::constitution