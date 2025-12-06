#pragma once
#include <uipc/common/type_define.h>
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyExternalBodyForce final : public IConstitution
{
    using Base = IConstitution;

  public:
    AffineBodyExternalBodyForce(const Json& config = default_config());
    ~AffineBodyExternalBodyForce();

    /**
     * @brief Apply external force (12D generalized force) to affine body instances
     * @param sc SimplicialComplex representing affine body geometry
     * @param force 12D generalized force vector: [fx, fy, fz, f_a11, f_a12, f_a13, f_a21, f_a22, f_a23, f_a31, f_a32, f_a33]
     *              where f is the 3D translational force and f_a is the 9D affine force
     */
    void apply_to(geometry::SimplicialComplex& sc, const Vector12& force);

    /**
     * @brief Apply external translational force to affine body instances (only translational force, affine force = 0)
     * @param sc SimplicialComplex representing affine body geometry
     * @param force 3D translational force vector
     */
    void apply_to(geometry::SimplicialComplex& sc, const Vector3& force);

    static Json default_config();

  protected:
    virtual U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
