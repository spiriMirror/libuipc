#pragma once
#include <uipc/constitution/inter_affine_body_constitution.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyPrismaticJoint final : public InterAffineBodyConstitution
{
  public:
    using SlotTuple =
        std::tuple<S<geometry::SimplicialComplexSlot>, S<geometry::SimplicialComplexSlot>>;

    static Json default_config();

    AffineBodyPrismaticJoint(const Json& config = default_config());

    virtual ~AffineBodyPrismaticJoint();

    /**
     * @brief Deprecated: Apply prismatic joint to edges connecting affine bodies.
     * 
     * This method is deprecated. Use the new apply_to methods with instance IDs for multi-instance support.
     * 
     * @param edges The simplicial complex containing the edges representing the joints.
     * @param geo_slots Pairs of geometry slots representing the connected bodies.
     * @param strength_ratio The strength ratio of the joint constraint (default: 100).
     */
    // Deprecated: Use the new apply_to method with instance IDs for multi-instance support
    void apply_to(geometry::SimplicialComplex& edges,
                  span<SlotTuple>              geo_slots,
                  Float                        strength_ratio = Float{100});
    
    /**
     * @brief Apply prismatic joint to edges connecting affine bodies (single-instance mode).
     * 
     * This method assumes each geometry has exactly one instance (instance 0).
     * All joints use the same strength ratio.
     * 
     * @param edges The simplicial complex containing the edges representing the joints.
     * @param l_geo_slots Left geometry slots for each joint.
     * @param r_geo_slots Right geometry slots for each joint.
     * @param strength_ratio The strength ratio of the joint constraint applied to all joints (default: 100).
     */
    void apply_to(geometry::SimplicialComplex& edges,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  Float                                  strength_ratio = Float{100});

    /**
     * @brief Apply prismatic joint to edges connecting affine bodies (multi-instance mode).
     * 
     * This method supports geometries with multiple instances. Each joint can specify
     * which instance of each geometry to connect, and can have its own strength ratio.
     * 
     * @param edges The simplicial complex containing the edges representing the joints.
     * @param l_geo_slots Left geometry slots for each joint.
     * @param l_instance_id Instance IDs for the left geometries (must be in range [0, instances().size())).
     * @param r_geo_slots Right geometry slots for each joint.
     * @param r_instance_id Instance IDs for the right geometries (must be in range [0, instances().size())).
     * @param strength_ratio The strength ratio for each joint (one per edge).
     */
    void apply_to(geometry::SimplicialComplex& edges,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<IndexT>                           l_instance_id,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  span<IndexT>                           r_instance_id,
                  span<Float>                            strength_ratio);

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
