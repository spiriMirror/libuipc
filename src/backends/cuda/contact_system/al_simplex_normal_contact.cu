#include <contact_system/al_simplex_normal_contact.h>
#include <contact_system/al_contact_function.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda {
    void ALSimplexNormalContact::do_build(ContactReporter::BuildInfo &info) {
        m_impl.global_contact_manager   = require<GlobalContactManager>();
        m_impl.global_vertex_manager    = require<GlobalVertexManager>();
        m_impl.global_surf_manager      = require<GlobalSimplicialSurfaceManager>();
        m_impl.global_active_set_manager = require<GlobalActiveSetManager>();
    }

    void ALSimplexNormalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo &info) {
        auto& active_set = m_impl.global_active_set_manager;

        if (!active_set->is_enabled()) {
            info.energy_count(0);
            return;
        }

        info.energy_count(active_set->PTs().size() + active_set->EEs().size());
    }

    void ALSimplexNormalContact::do_report_gradient_hessian_extent(
        GlobalContactManager::GradientHessianExtentInfo &info) {
        auto& active_set = m_impl.global_active_set_manager;

        if (!active_set->is_enabled()) {
            info.gradient_count(0);
            info.hessian_count(0);
            return;
        }

        info.gradient_count(4 * (active_set->PTs().size() + active_set->EEs().size()));
        info.hessian_count(16 * (active_set->PTs().size() + active_set->EEs().size()));
    }

    void ALSimplexNormalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo &info) {
        using namespace muda;
        using namespace sym::al_simplex_contact;
        auto& active_set = global_active_set_manager;

        if (!active_set->is_enabled()) return;

        auto PT_size = active_set->PTs().size(), EE_size = active_set->EEs().size();
        auto PT_energies = info.energies().subview(0, PT_size);
        auto EE_energies = info.energies().subview(PT_size, EE_size);

        auto x = global_vertex_manager->positions();
        auto PTs = active_set->PTs(), EEs = active_set->EEs();
        auto PT_d0 = active_set->PT_d0(), EE_d0 = active_set->EE_d0();
        auto PT_cnt = active_set->PT_cnt(), EE_cnt = active_set->EE_cnt();
        auto PT_d_grad = active_set->PT_d_grad();
        auto EE_d_grad = active_set->EE_d_grad();

        ParallelFor().file_line(__FILE__, __LINE__).apply(PT_size, [
            mu = active_set->mu(), decay = active_set->decay_factor(),
            PTs = PTs.cviewer().name("PTs"),
            cnt = PT_cnt.cviewer().name("cnt"),
            d0 = PT_d0.cviewer().name("d0"),
            d_grad = PT_d_grad.cviewer().name("d_grad"),
            x = x.cviewer().name("x"),
            Es = PT_energies.viewer().name("Es")
        ] __device__ (int idx) mutable {
            auto PT = PTs(idx);
            Es(idx) = penalty_energy(pow(decay, cnt(idx)) * mu, d0(idx), d_grad(idx),
                                     x(PT(0)), x(PT(1)), x(PT(2)), x(PT(3)));
        });

        ParallelFor().file_line(__FILE__, __LINE__).apply(EE_size, [
            mu = active_set->mu(), decay = active_set->decay_factor(),
            EEs = EEs.cviewer().name("EEs"),
            cnt = EE_cnt.cviewer().name("cnt"),
            d0 = EE_d0.cviewer().name("d0"),
            d_grad = EE_d_grad.cviewer().name("d_grad"),
            x = x.cviewer().name("x"),
            Es = EE_energies.viewer().name("Es")
        ] __device__ (int idx) mutable {
            auto EE = EEs(idx);
            Es(idx) = penalty_energy(pow(decay, cnt(idx)) * mu, d0(idx), d_grad(idx),
                                     x(EE(0)), x(EE(1)), x(EE(2)), x(EE(3)));
        });
    }

    void ALSimplexNormalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo &info) {
        using namespace muda;
        using namespace sym::al_simplex_contact;
        auto& active_set = global_active_set_manager;

        if (!active_set->is_enabled()) return;

        auto PT_size = active_set->PTs().size(), EE_size = active_set->EEs().size();
        auto PT_grad = info.gradients().subview(0, PT_size*4);
        auto EE_grad = info.gradients().subview(PT_size*4, EE_size*4);
        auto PT_hess = info.hessians().subview(0, PT_size*16);
        auto EE_hess = info.hessians().subview(PT_size*16, EE_size*16);

        auto x = global_vertex_manager->positions();
        auto PTs = active_set->PTs(), EEs = active_set->EEs();
        auto PT_d0 = active_set->PT_d0(), EE_d0 = active_set->EE_d0();
        auto PT_cnt = active_set->PT_cnt(), EE_cnt = active_set->EE_cnt();
        auto PT_d_grad = active_set->PT_d_grad();
        auto EE_d_grad = active_set->EE_d_grad();

        ParallelFor().file_line(__FILE__, __LINE__).apply(PT_size, [
            mu = active_set->mu(), decay = active_set->decay_factor(),
            PTs = PTs.cviewer().name("PTs"),
            cnt = PT_cnt.cviewer().name("cnt"),
            d0 = PT_d0.cviewer().name("d0"),
            d_grad = PT_d_grad.cviewer().name("d_grad"),
            x = x.cviewer().name("x"),
            Gs = PT_grad.viewer().name("Gs"),
            Hs = PT_hess.viewer().name("Hs")
        ] __device__ (int idx) mutable {
            auto PT = PTs(idx);
            Vector12 G;
            Matrix12x12 H;
            penalty_gradient_hessian(pow(decay, cnt(idx)) * mu, d0(idx), d_grad(idx),
                 x(PT(0)), x(PT(1)), x(PT(2)), x(PT(3)), G, H);

            DoubletVectorAssembler DVA{Gs};
            DVA.segment<4>(idx * 4).write(PT, G);

            TripletMatrixAssembler TMA{Hs};
            TMA.block<4, 4>(idx * 16).write(PT, H);
        });

        ParallelFor().file_line(__FILE__, __LINE__).apply(EE_size, [
            mu = active_set->mu(), decay = active_set->decay_factor(),
            EEs = EEs.cviewer().name("EEs"),
            cnt = EE_cnt.cviewer().name("cnt"),
            d0 = EE_d0.cviewer().name("d0"),
            d_grad = EE_d_grad.cviewer().name("d_grad"),
            x = x.cviewer().name("x"),
            Gs = EE_grad.viewer().name("Gs"),
            Hs = EE_hess.viewer().name("Hs")
        ] __device__ (int idx) mutable {
            auto EE = EEs(idx);
            Vector12 G;
            Matrix12x12 H;
            penalty_gradient_hessian(pow(decay, cnt(idx)) * mu, d0(idx), d_grad(idx),
                 x(EE(0)), x(EE(1)), x(EE(2)), x(EE(3)), G, H);

            DoubletVectorAssembler DVA{Gs};
            DVA.segment<4>(idx * 4).write(EE, G);

            TripletMatrixAssembler TMA{Hs};
            TMA.block<4, 4>(idx * 16).write(EE, H);
        });
    }

    void ALSimplexNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo &info) {
        m_impl.do_compute_energy(info);
    }

    void ALSimplexNormalContact::do_assemble(GlobalContactManager::GradientHessianInfo &info) {
        m_impl.do_assemble(info);
    }

    REGISTER_SIM_SYSTEM(ALSimplexNormalContact);
}
