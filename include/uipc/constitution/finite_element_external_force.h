#pragma once
#include <uipc/common/type_define.h>
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API FiniteElementExternalForce final : public IConstitution
{
    using Base = IConstitution;

  public:
    FiniteElementExternalForce(const Json& config = default_config());
    ~FiniteElementExternalForce();

    /**
     * @brief Apply external force (3D) to finite element vertices
     * @param sc SimplicialComplex representing finite element geometry
     * @param force 3D force vector applied uniformly to all vertices
     * @param enable Enable external force on all vertices (default: true).
     * Sets `external_force/is_constrained`; does not modify `is_constrained`
     * used by soft position constraints. Change the dedicated attribute per
     * vertex in the Animator to toggle forces. Reapplying resets this mask.
     */
    void apply_to(geometry::SimplicialComplex& sc, const Vector3& force, bool enable = true);

    static Json default_config();

  protected:
    virtual U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
