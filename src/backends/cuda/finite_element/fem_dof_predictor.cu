#include <finite_element/finite_element_method.h>
#include <dof_predictor.h>

namespace uipc::backend::cuda
{
class FEMDofPredictor : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    FiniteElementMethod* finite_element_method = nullptr;

    virtual void do_build() override
    {
        finite_element_method = &require<FiniteElementMethod>();

        auto& dof_predictor = require<DofPredictor>();

        dof_predictor.on_predict(*this,
                                 [this](DofPredictor::PredictInfo& info)
                                 { compute_x_tilde(info); });

        dof_predictor.on_compute_velocity(*this,
                                          [this](DofPredictor::ComputeVelocityInfo& info)
                                          { compute_velocity(info); });
    }

    void compute_x_tilde(DofPredictor::PredictInfo& info)
    {
        using namespace muda;

        auto& fem        = finite_element_method->m_impl;
        auto& xs         = fem.xs;
        auto& is_fixed   = fem.is_fixed;
        auto& is_dynamic = fem.is_dynamic;
        auto& x_prevs    = fem.x_prevs;
        auto& vs         = fem.vs;
        auto& x_tildes   = fem.x_tildes;
        auto& gravities  = fem.gravities;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(xs.size(),
                   [is_fixed   = is_fixed.cviewer().name("fixed"),
                    is_dynamic = is_dynamic.cviewer().name("is_dynamic"),
                    x_prevs    = x_prevs.cviewer().name("x_prevs"),
                    vs         = vs.cviewer().name("vs"),
                    x_tildes   = x_tildes.viewer().name("x_tildes"),
                    gravities  = gravities.cviewer().name("gravities"),
                    dt         = info.dt()] __device__(int i) mutable
                   {
                       const Vector3& x_prev = x_prevs(i);
                       const Vector3& v      = vs(i);

                       // 0) fixed: x_tilde = x_prev
                       Vector3 x_tilde = x_prev;

                       if(!is_fixed(i))
                       {
                           const Vector3& g = gravities(i);

                           // 1) static problem: x_tilde = x_prev + g * dt * dt
                           x_tilde += g * dt * dt;

                           // 2) dynamic problem: x_tilde = x_prev + v * dt + g * dt * dt
                           if(is_dynamic(i))
                           {
                               x_tilde += v * dt;
                           }
                       }

                       x_tildes(i) = x_tilde;
                   });
    }

    void compute_velocity(DofPredictor::ComputeVelocityInfo& info)
    {
        using namespace muda;

        auto& fem     = finite_element_method->m_impl;
        auto& xs      = fem.xs;
        auto& vs      = fem.vs;
        auto& x_prevs = fem.x_prevs;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(xs.size(),
                   [xs      = xs.cviewer().name("xs"),
                    vs      = vs.viewer().name("vs"),
                    x_prevs = x_prevs.viewer().name("x_prevs"),
                    dt      = info.dt()] __device__(int i) mutable
                   {
                       Vector3&       v      = vs(i);
                       Vector3&       x_prev = x_prevs(i);
                       const Vector3& x      = xs(i);

                       v = (x - x_prev) * (1.0 / dt);

                       x_prev = x;
                   });
    }
};

REGISTER_SIM_SYSTEM(FEMDofPredictor);
}  // namespace uipc::backend::cuda
