#pragma once
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyDrivingPrismaticJoint final : public Constraint
{
    using Base = Constraint;

  public:
    AffineBodyDrivingPrismaticJoint(const Json& config = default_config());

    ~AffineBodyDrivingPrismaticJoint() override;

    /**
     * @brief Apply driving prismatic joint to edges connecting affine bodies (single-instance mode).
     * 
     * This method assumes each geometry has exactly one instance (instance 0).
     * All joints use the same strength ratio.
     * 
     * @param sc The simplicial complex containing the edges representing the joints.
     * @param strength_ratio The strength ratio of the joint constraint applied to all joints (default: 100).
     */
    void apply_to(geometry::SimplicialComplex& sc, Float strength_ratio = Float{100});


    /**
     * @brief Apply driving prismatic joint to edges connecting affine bodies (multi-instance mode).
     * 
     * This method supports geometries with multiple instances. Each joint can specify
     * which instance of each geometry to connect, and can have its own strength ratio.
     * @param sc The simplicial complex containing the edges representing the joints.
     * @param strength_ratios The strength ratio for each joint (one per edge).
     */
    void apply_to(geometry::SimplicialComplex& sc, span<Float> strength_ratios);

    static Json default_config();

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution