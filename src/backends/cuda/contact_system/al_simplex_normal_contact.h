#pragma once

#include <contact_system/contact_reporter.h>
#include <line_search/line_searcher.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <active_set_system/global_active_set_manager.h>

namespace uipc::backend::cuda
{
class ALSimplexNormalContact : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;
    constexpr static SizeT PTHalfHessianSize = 4 * (4 + 1) / 2;  // 4 vertices, symmetric matrix
    constexpr static SizeT EEHalfHessianSize = 4 * (4 + 1) / 2;  // 4 vertices, symmetric matrix

    class Impl
    {
      public:
        SimSystemSlot<GlobalContactManager>           global_contact_manager;
        SimSystemSlot<GlobalVertexManager>            global_vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager> global_surf_manager;
        SimSystemSlot<GlobalActiveSetManager>         global_active_set_manager;

        void do_compute_energy(GlobalContactManager::EnergyInfo& info);
        void do_assemble(GlobalContactManager::GradientHessianInfo& info);
    };

  private:
    virtual void do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info) override final;

    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) override final;

    virtual void do_report_gradient_hessian_extent(
        GlobalContactManager::GradientHessianExtentInfo& info) override final;

    virtual void do_assemble(GlobalContactManager::GradientHessianInfo& info) override final;

    virtual void do_build(ContactReporter::BuildInfo& info) override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
