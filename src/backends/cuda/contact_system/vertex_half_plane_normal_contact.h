#pragma once
#include <contact_system/contact_reporter.h>
#include <line_search/line_searcher.h>
#include <contact_system/contact_coeff.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>

namespace uipc::backend::cuda
{
class GlobalTrajectoryFilter;
class VertexHalfPlaneTrajectoryFilter;

class VertexHalfPlaneNormalContact : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;

    class Impl;


    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        muda::CBuffer2DView<ContactCoeff> contact_tabular() const;
        muda::CBufferView<Vector2i>       PHs() const;
        muda::CBufferView<Vector3>        positions() const;
        muda::CBufferView<Vector3>        prev_positions() const;
        muda::CBufferView<Vector3>        rest_positions() const;
        muda::CBufferView<Float>          thicknesses() const;
        muda::CBufferView<IndexT>         contact_element_ids() const;
        muda::CBufferView<IndexT>         subscene_element_ids() const;
        Float                             d_hat() const;
        muda::CBufferView<Float>          d_hats() const;
        Float                             dt() const;
        Float                             eps_velocity() const;
        IndexT                            half_plane_vertex_offset() const;

      private:
        friend class VertexHalfPlaneNormalContact;
        Impl* m_impl;
    };

    class ContactInfo : public BaseInfo
    {
      public:
        ContactInfo(Impl* impl) noexcept
            : BaseInfo(impl)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }
        auto gradient_only() const noexcept { return m_gradient_only; }

      private:
        friend class VertexHalfPlaneNormalContact;

        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
        bool                              m_gradient_only = false;
    };

    class BuildInfo
    {
      public:
    };

    class EnergyInfo : public BaseInfo
    {
      public:
        EnergyInfo(Impl* impl) noexcept
            : BaseInfo(impl)
        {
        }

        muda::BufferView<Float> energies() const noexcept;

      private:
        friend class VertexHalfPlaneNormalContact;
        muda::BufferView<Float> m_energies;
    };

    class Impl
    {
      public:
        void compute_energy(EnergyInfo& info);

        SimSystemSlot<GlobalTrajectoryFilter> global_trajectory_filter;
        SimSystemSlot<GlobalContactManager>   global_contact_manager;
        SimSystemSlot<GlobalVertexManager>    global_vertex_manager;
        SimSystemSlot<VertexHalfPlaneTrajectoryFilter> veretx_half_plane_trajectory_filter;
        SimSystemSlot<HalfPlaneVertexReporter> vertex_reporter;

        SizeT PH_count = 0;
        Float dt       = 0.0;

        muda::CBufferView<Float>           energies;
        muda::CDoubletVectorView<Float, 3> gradients;
        muda::CTripletMatrixView<Float, 3> hessians;
    };

    muda::CBufferView<Vector2i>        PHs() const noexcept;
    muda::CBufferView<Float>           energies() const noexcept;
    muda::CDoubletVectorView<Float, 3> gradients() const noexcept;
    muda::CTripletMatrixView<Float, 3> hessians() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)           = 0;
    virtual void do_compute_energy(EnergyInfo& info) = 0;
    virtual void do_assemble(ContactInfo& info)      = 0;

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
