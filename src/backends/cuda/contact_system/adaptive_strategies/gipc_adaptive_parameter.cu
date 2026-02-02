#include <contact_system/adaptive_contact_parameter_reporter.h>
#include <uipc/builtin/constants.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <uipc/common/zip.h>
#include <uipc/common/enumerate.h>
#include <linear_system/global_linear_system.h>

namespace uipc::backend::cuda
{
// ref: https://github.com/KemengHuang/Stiff-GIPC
class GIPCAdaptiveParameterReporter : public AdaptiveContactParameterReporter
{
  public:
    using AdaptiveContactParameterReporter::AdaptiveContactParameterReporter;

    SimSystemSlot<GlobalContactManager>           contact_manager;
    SimSystemSlot<GlobalVertexManager>            vertex_manager;
    SimSystemSlot<GlobalSimplicialSurfaceManager> surface_manager;
    SimSystemSlot<GlobalLinearSystem>             linear_system;
    SimSystemSlot<GlobalDyTopoEffectManager>      dytopo_effect_manager;

    void do_build(BuildInfo& info) override
    {
        auto scene = world().scene();
        auto kappas = scene.contact_tabular().contact_models().find<Float>("resistance");
        auto kappa_view = kappas->view();
        auto min_kappa  = std::ranges::min(kappa_view);
        if(min_kappa >= 0.0)
            throw SimSystemException{"No Adaptive Kappa is needed"};

        contact_manager       = require<GlobalContactManager>();
        vertex_manager        = require<GlobalVertexManager>();
        surface_manager       = require<GlobalSimplicialSurfaceManager>();
        linear_system         = require<GlobalLinearSystem>();
        dytopo_effect_manager = require<GlobalDyTopoEffectManager>();
    }

    muda::DeviceBuffer<Vector2i> adaptive_topos;

    virtual void do_init(InitInfo& info) override
    {
        auto scene          = world().scene();
        auto N              = scene.contact_tabular().element_count();
        auto contact_models = scene.contact_tabular().contact_models();
        auto kappas         = contact_models.find<Float>("resistance");
        auto topos          = contact_models.find<Vector2i>("topo");

        auto kappa_view = kappas->view();
        auto topo_view  = topos->view();

        std::vector<Vector2i> h_adaptive_topos;
        h_adaptive_topos.reserve(topos->size());

        // copy if a contact model is adaptive
        for(auto [i, topo] : uipc::enumerate(topo_view))
        {
            if(kappa_view[i] < 0.0)
                h_adaptive_topos.push_back(topo);
        }

        adaptive_topos.copy_from(h_adaptive_topos);

        contact_gradient.resize(linear_system->dof_count());
        non_contact_gradient.resize(linear_system->dof_count());
    }

    muda::DeviceDenseVector<Float> contact_gradient;
    muda::DeviceDenseVector<Float> non_contact_gradient;

    void compute_gradient()
    {
        auto _compute_gradient = [this](EnergyComponentFlags         flags,
                                        muda::DenseVectorView<Float> gradient)
        {
            // 1. Compute dytopo effect
            GlobalDyTopoEffectManager::ComputeDyTopoEffectInfo dytopo_effect_info;
            dytopo_effect_info.component_flags(flags);
            dytopo_effect_info.gradient_only(true);
            dytopo_effect_manager->compute_dytopo_effect(dytopo_effect_info);
            // 2. Compute the gradient
            GlobalLinearSystem::ComputeGradientInfo grad_info;

            grad_info.buffer_view(gradient);
            grad_info.flags(flags);
            linear_system->compute_gradient(grad_info);
        };

        contact_gradient.resize(linear_system->dof_count());
        non_contact_gradient.resize(linear_system->dof_count());

        // compute contact gradient
        _compute_gradient(EnergyComponentFlags::Contact, contact_gradient);
        // compute non-contact gradient
        _compute_gradient(EnergyComponentFlags::Complement, non_contact_gradient);
    }

    virtual void do_compute_parameters(AdaptiveParameterInfo& info) override
    {
        compute_gradient();

        using namespace muda;

        Float newKappa = 0.0;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(adaptive_topos.size(),
                   [adaptive_topos = adaptive_topos.viewer().name("adaptive_topos"),
                    contact_tabular = info.contact_tabular().viewer().name("contact_tabular"),
                    newKappa = newKappa] __device__(IndexT I) mutable
                   {
                       Vector2i topo  = adaptive_topos(I);
                       auto&    coefL = contact_tabular(topo.x(), topo.y());
                       auto&    coefR = contact_tabular(topo.y(), topo.x());

                       coefL.kappa = newKappa;
                       coefR.kappa = newKappa;
                   });
    }
};

REGISTER_SIM_SYSTEM(GIPCAdaptiveParameterReporter);
}  // namespace uipc::backend::cuda