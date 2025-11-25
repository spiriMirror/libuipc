#pragma once
#include <uipc/common/type_define.h>
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyExternalWrench final : public IConstitution
{
    using Base = IConstitution;

  public:
    AffineBodyExternalWrench(const Json& config = default_config());
    ~AffineBodyExternalWrench();

    /**
     * @brief Apply external wrench (generalized force) to affine body instances
     * @param sc SimplicialComplex representing affine body geometry
     * @param wrench 12D generalized force vector: [fx, fy, fz, dS11/dt, dS12/dt, ..., dS33/dt]
     *               where f is the 3D translational force and dS/dt is the flattened shape velocity derivative (9D)
     */
    void apply_to(geometry::SimplicialComplex& sc, const Vector12& wrench);

    /**
     * @brief Apply external force to affine body instances (only translational force, shape velocity derivative = 0)
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
