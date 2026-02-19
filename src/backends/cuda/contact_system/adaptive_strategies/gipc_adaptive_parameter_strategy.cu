#include <contact_system/adaptive_contact_parameter_reporter.h>
#include <uipc/builtin/constants.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <uipc/common/zip.h>
#include <uipc/common/enumerate.h>
#include <linear_system/global_linear_system.h>
#include <contact_system/contact_models/codim_ipc_contact_function.h>

namespace uipc::backend::cuda
{
// ref: https://github.com/KemengHuang/Stiff-GIPC
class GIPCAdaptiveParameterStrategy : public AdaptiveContactParameterReporter
{
  public:
    using AdaptiveContactParameterReporter::AdaptiveContactParameterReporter;

    SimSystemSlot<GlobalContactManager>           contact_manager;
    SimSystemSlot<GlobalVertexManager>            vertex_manager;
    SimSystemSlot<GlobalSimplicialSurfaceManager> surface_manager;
    SimSystemSlot<GlobalLinearSystem>             linear_system;
    SimSystemSlot<GlobalDyTopoEffectManager>      dytopo_effect_manager;

    std::vector<IndexT>                   h_adaptive_kappa_index;
    muda::DeviceBuffer<Vector2i>          adaptive_topos;
    S<muda::DeviceBuffer2D<ContactCoeff>> test_contact_tabular;

    Float min_kappa  = 0.0;
    Float init_kappa = 0.0;
    Float max_kappa  = 0.0;
    Float new_kappa  = 0.0;

    void do_build(BuildInfo& info) override
    {
        auto scene = world().scene();
        {
            auto kappas = scene.contact_tabular().contact_models().find<Float>("resistance");
            auto kappa_view      = kappas->view();
            auto found_min_kappa = std::ranges::min(kappa_view);
            if(found_min_kappa >= 0.0)
                throw SimSystemException{"No Adaptive Kappa is detected"};
        }

        contact_manager       = require<GlobalContactManager>();
        vertex_manager        = require<GlobalVertexManager>();
        surface_manager       = require<GlobalSimplicialSurfaceManager>();
        linear_system         = require<GlobalLinearSystem>();
        dytopo_effect_manager = require<GlobalDyTopoEffectManager>();

        min_kappa = scene.config().find<Float>("contact/adaptive/min_kappa")->view()[0];
        init_kappa =
            scene.config().find<Float>("contact/adaptive/init_kappa")->view()[0];
        max_kappa = scene.config().find<Float>("contact/adaptive/max_kappa")->view()[0];

        on_write_scene([this] { write_scene(); });
    }


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
        h_adaptive_kappa_index.reserve(topos->size());

        // copy if a contact model is adaptive
        for(auto [i, topo] : uipc::enumerate(topo_view))
        {
            if(kappa_view[i] < 0.0)
            {
                h_adaptive_topos.push_back(topo);
                h_adaptive_kappa_index.push_back(i);
            }
        }

        adaptive_topos.copy_from(h_adaptive_topos);

        contact_gradient.resize(linear_system->dof_count());
        non_contact_gradient.resize(linear_system->dof_count());

        // initialize test contact tabular
        // non-adaptive kappa to 0.0 (don't contribute)
        // adaptive kappa to 1.0 (contribute)

        test_contact_tabular = std::make_shared<muda::DeviceBuffer2D<ContactCoeff>>();
        test_contact_tabular->resize({N, N});
        test_contact_tabular->fill(ContactCoeff{0.0f});

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(adaptive_topos.size(),
                   [adaptive_topos = adaptive_topos.viewer().name("adaptive_topos"),
                    contact_tabular = test_contact_tabular->viewer().name(
                        "contact_tabular")] __device__(IndexT I) mutable
                   {
                       Vector2i topo  = adaptive_topos(I);
                       auto&    coefL = contact_tabular(topo.x(), topo.y());
                       auto&    coefR = contact_tabular(topo.y(), topo.x());
                       coefL.kappa    = 1.0;
                       coefR.kappa    = 1.0;
                   });

        //auto             d_hats = vertex_manager->d_hats();
        //DeviceVar<Float> min_d_hat;
        //DeviceReduce().Min(d_hats.data(), min_d_hat.data(), d_hats.size());
    }

    muda::DeviceDenseVector<Float> contact_gradient;
    muda::DeviceDenseVector<Float> non_contact_gradient;

    void compute_gradient(AdaptiveParameterInfo& info)
    {
        // set a test contact tabular
        auto original_contact_tabular = info.exchange_contact_tabular(test_contact_tabular);

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


        // recover original contact tabular
        info.exchange_contact_tabular(original_contact_tabular);
    }

    virtual void do_compute_parameters(AdaptiveParameterInfo& info) override
    {

        using namespace muda;
        using namespace sym::codim_ipc_contact;

        compute_gradient(info);

        std::vector<Float> h_contact_gradient;
        contact_gradient.copy_to(h_contact_gradient);

        std::vector<Float> h_non_contact_gradient;
        non_contact_gradient.copy_to(h_non_contact_gradient);

        auto& ctx = linear_system->ctx();

        Float contact2 = ctx.dot(contact_gradient.cview(), contact_gradient.cview());
        Float proj = ctx.dot(contact_gradient.cview(), non_contact_gradient.cview());

        // 1. No contact contribution -> keep initial kappa
        Float proj_kappa = (contact2 == 0.0) ? init_kappa : -proj / contact2;

        // 2. Clamp to [min_kappa, max_kappa]
        new_kappa = std::clamp(proj_kappa, min_kappa, max_kappa);

        logger::info(R"(Adaptive Contact Parameter: > Kappa = {:e}
* ProjKappa = {:e}, Contact^2 = {:e}, Contact.NonContact = {:e}
* MinKappa = {:e}, InitKappa={:e}, MaxKappa = {:e})",
                     new_kappa,
                     proj_kappa,
                     contact2,
                     proj,
                     min_kappa,
                     init_kappa,
                     max_kappa);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(adaptive_topos.size(),
                   [adaptive_topos = adaptive_topos.viewer().name("adaptive_topos"),
                    contact_tabular = info.contact_tabular().viewer().name("contact_tabular"),
                    new_kappa = new_kappa] __device__(IndexT I) mutable
                   {
                       Vector2i topo  = adaptive_topos(I);
                       auto&    coefL = contact_tabular(topo.x(), topo.y());
                       auto&    coefR = contact_tabular(topo.y(), topo.x());

                       coefL.kappa = new_kappa;
                       coefR.kappa = new_kappa;
                   });
    }

    vector<Float> h_kappas;

    void write_scene()
    {
        auto scene = world().scene();
        auto kappa = scene.contact_tabular().contact_models().find<Float>("resistance");
        auto kappa_view = view(*kappa);
        for(auto index : h_adaptive_kappa_index)
            kappa_view[index] = new_kappa;
    }
};

REGISTER_SIM_SYSTEM(GIPCAdaptiveParameterStrategy);
}  // namespace uipc::backend::cuda