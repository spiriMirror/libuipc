#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <muda/ext/linear_system.h>
#include <contact_system/contact_coeff.h>
#include <algorithm/matrix_converter.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class ContactReporter;
class ContactReceiver;
class GlobalTrajectoryFilter;
class AdaptiveContactParameterReporter;

class GlobalContactManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    using GradientHessianExtentInfo = GlobalDyTopoEffectManager::GradientHessianExtentInfo;

    using GradientHessianInfo = GlobalDyTopoEffectManager::GradientHessianInfo;

    using EnergyExtentInfo = GlobalDyTopoEffectManager::EnergyExtentInfo;

    using EnergyInfo = GlobalDyTopoEffectManager::EnergyInfo;

    class Impl;

    class AdaptiveParameterInfo
    {
      public:
        AdaptiveParameterInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        muda::Buffer2DView<ContactCoeff> contact_tabular() const noexcept;

        S<muda::DeviceBuffer2D<ContactCoeff>> exchange_contact_tabular(
            S<muda::DeviceBuffer2D<ContactCoeff>> new_buffer) const noexcept;

      private:
        Impl* m_impl;
    };

    class Impl
    {
      public:
        void  init(WorldVisitor& world);
        void  _build_contact_tabular(WorldVisitor& world);
        void  _build_subscene_tabular(WorldVisitor& world);
        void  compute_d_hat();
        void  compute_adaptive_kappa();
        Float compute_cfl_condition();

        SimSystemSlot<GlobalVertexManager>    global_vertex_manager;
        SimSystemSlot<GlobalTrajectoryFilter> global_trajectory_filter;

        bool cfl_enabled = false;

        S<muda::DeviceBuffer2D<ContactCoeff>> contact_tabular;

        vector<ContactCoeff>         h_contact_tabular;
        vector<IndexT>               h_contact_mask_tabular;
        vector<IndexT>               h_subcene_mask_tabular;
        muda::DeviceBuffer2D<IndexT> contact_mask_tabular;
        muda::DeviceBuffer2D<IndexT> subscene_mask_tabular;
        Float                        reserve_ratio = 1.1;

        Float d_hat        = 0.0;
        Float kappa        = 0.0;
        Float dt           = 0.0;
        Float eps_velocity = 0.0;


        /***********************************************************************
        *                     Global Vertex Contact Info                       *
        ***********************************************************************/

        muda::DeviceBuffer<IndexT> vert_is_active_contact;
        muda::DeviceBuffer<Float>  vert_disp_norms;
        muda::DeviceVar<Float>     max_disp_norm;

        SimSystemSlotCollection<ContactReporter> contact_reporters;
        SimSystemSlotCollection<ContactReceiver> contact_receivers;
        SimSystemSlot<AdaptiveContactParameterReporter> adaptive_contact_parameter_reporter;
    };

    Float d_hat() const;
    Float eps_velocity() const;
    bool  cfl_enabled() const;

    muda::CBuffer2DView<ContactCoeff> contact_tabular() const noexcept;
    muda::CBuffer2DView<IndexT>       contact_mask_tabular() const noexcept;
    muda::CBuffer2DView<IndexT>       subscene_mask_tabular() const noexcept;



  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    friend class ContactLineSearchReporter;
    friend class GlobalTrajectoryFilter;
    friend class ContactExporterManager;

    void init();

    void compute_adaptive_parameters();

    Float compute_cfl_condition();

    friend class ContactReporter;
    void add_reporter(ContactReporter* reporter);

    friend class AdaptiveContactParameterReporter;
    void add_reporter(AdaptiveContactParameterReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda