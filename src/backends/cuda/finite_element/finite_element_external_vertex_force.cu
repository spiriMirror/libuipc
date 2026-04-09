#include <finite_element/finite_element_external_force_reporter.h>
#include <finite_element/constraints/finite_element_external_vertex_force_constraint.h>
#include <finite_element/finite_element_method.h>
#include <muda/ext/eigen/atomic.h>

namespace uipc::backend::cuda
{
/**
 * @brief Scatter-add external forces from constraint buffers into the
 *        per-vertex external force buffer on FiniteElementMethod.
 */
class FiniteElementExternalVertexForce final : public FiniteElementExternalForceReporter
{
  public:
    static constexpr U64 UID = 671;

    using FiniteElementExternalForceReporter::FiniteElementExternalForceReporter;

    SimSystemSlot<FiniteElementMethod>                          finite_element_method;
    SimSystemSlot<FiniteElementExternalVertexForceConstraint>   constraint;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method = require<FiniteElementMethod>();
        constraint            = require<FiniteElementExternalVertexForceConstraint>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override {}

    void do_step(ExternalForceInfo& info) override
    {
        SizeT force_count = constraint->forces().size();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(force_count,
                   [forces     = info.external_forces().viewer().name("forces"),
                    vertex_ids = constraint->vertex_ids().viewer().name("vertex_ids"),
                    vertex_forces = constraint->forces().viewer().name(
                        "vertex_forces")] __device__(int i) mutable
                   {
                       auto vid = vertex_ids(i);
                       eigen::atomic_add(forces(vid), vertex_forces(i));
                   });
    }
};

REGISTER_SIM_SYSTEM(FiniteElementExternalVertexForce);
}  // namespace uipc::backend::cuda
