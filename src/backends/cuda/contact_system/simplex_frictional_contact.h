#pragma once
#include <contact_system/contact_reporter.h>
#include <line_search/line_searcher.h>
#include <contact_system/contact_coeff.h>
#include <collision_detection/simplex_trajectory_filter.h>

namespace uipc::backend::cuda
{
class SimplexFrictionalContact : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;
    constexpr static SizeT PTHalfHessianSize = 4 * (4 + 1) / 2;  // 4 vertices, symmetric matrix
    constexpr static SizeT EEHalfHessianSize = 4 * (4 + 1) / 2;  // 4 vertices, symmetric matrix
    constexpr static SizeT PEHalfHessianSize = 3 * (3 + 1) / 2;  // 3 vertices, symmetric matrix
    constexpr static SizeT PPHalfHessianSize = 2 * (2 + 1) / 2;  // 2 vertices, symmetric matrix

    class Impl;

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular() const;
        cuda_tool::CBufferView<Vector4i>       friction_PTs() const;
        cuda_tool::CBufferView<Vector4i>       friction_EEs() const;
        cuda_tool::CBufferView<Vector3i>       friction_PEs() const;
        cuda_tool::CBufferView<Vector2i>       friction_PPs() const;
        cuda_tool::CBufferView<Vector3>        positions() const;
        cuda_tool::CBufferView<Vector3>        prev_positions() const;
        cuda_tool::CBufferView<Vector3>        rest_positions() const;
        cuda_tool::CBufferView<Float>          thicknesses() const;
        cuda_tool::CBufferView<IndexT>         contact_element_ids() const;
        Float                                  d_hat() const;
        cuda_tool::CBufferView<Float>          d_hats() const;
        Float                                  dt() const;
        Float                                  eps_velocity() const;

      private:
        friend class SimplexFrictionalContact;
        Impl* m_impl;
    };

    class ContactInfo : public BaseInfo
    {
      public:
        ContactInfo(Impl* impl) noexcept
            : BaseInfo(impl)
        {
        }
        auto friction_PT_gradients() const noexcept { return m_PT_gradients; }
        auto friction_PT_hessians() const noexcept { return m_PT_hessians; }
        auto friction_EE_gradients() const noexcept { return m_EE_gradients; }
        auto friction_EE_hessians() const noexcept { return m_EE_hessians; }
        auto friction_PE_gradients() const noexcept { return m_PE_gradients; }
        auto friction_PE_hessians() const noexcept { return m_PE_hessians; }
        auto friction_PP_gradients() const noexcept { return m_PP_gradients; }
        auto friction_PP_hessians() const noexcept { return m_PP_hessians; }
        bool gradient_only() const noexcept { return m_gradient_only; }

      private:
        friend class SimplexFrictionalContact;
        cuda_tool::DoubletVectorView<Float, 3> m_PT_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_PT_hessians;

        cuda_tool::DoubletVectorView<Float, 3> m_EE_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_EE_hessians;

        cuda_tool::DoubletVectorView<Float, 3> m_PE_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_PE_hessians;

        cuda_tool::DoubletVectorView<Float, 3> m_PP_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_PP_hessians;
        bool                                   m_gradient_only = false;
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

        cuda_tool::BufferView<Float> friction_PT_energies() const noexcept
        {
            return m_PT_energies;
        }
        cuda_tool::BufferView<Float> friction_EE_energies() const noexcept
        {
            return m_EE_energies;
        }
        cuda_tool::BufferView<Float> friction_PE_energies() const noexcept
        {
            return m_PE_energies;
        }
        cuda_tool::BufferView<Float> friction_PP_energies() const noexcept
        {
            return m_PP_energies;
        }

      private:
        friend class SimplexFrictionalContact;
        cuda_tool::BufferView<Float> m_PT_energies;
        cuda_tool::BufferView<Float> m_EE_energies;
        cuda_tool::BufferView<Float> m_PE_energies;
        cuda_tool::BufferView<Float> m_PP_energies;
    };

    class Impl
    {
      public:
        SimSystemSlot<GlobalTrajectoryFilter> global_trajectory_filter;
        SimSystemSlot<GlobalContactManager>   global_contact_manager;
        SimSystemSlot<GlobalVertexManager>    global_vertex_manager;

        SimSystemSlot<SimplexTrajectoryFilter> simplex_trajectory_filter;

        SizeT                                   PT_count = 0;
        SizeT                                   EE_count = 0;
        SizeT                                   PE_count = 0;
        SizeT                                   PP_count = 0;
        S<const geometry::AttributeSlot<Float>> dt_attr;

        cuda_tool::CBufferView<Float>           PT_energies;
        cuda_tool::CDoubletVectorView<Float, 3> PT_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> PT_hessians;

        cuda_tool::CBufferView<Float>           EE_energies;
        cuda_tool::CDoubletVectorView<Float, 3> EE_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> EE_hessians;

        cuda_tool::CBufferView<Float>           PE_energies;
        cuda_tool::CDoubletVectorView<Float, 3> PE_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> PE_hessians;

        cuda_tool::CBufferView<Float>           PP_energies;
        cuda_tool::CDoubletVectorView<Float, 3> PP_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> PP_hessians;
    };

    cuda_tool::CBufferView<Vector4i>        PTs() const;
    cuda_tool::CBufferView<Float>           PT_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> PT_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> PT_hessians() const;

    cuda_tool::CBufferView<Vector4i>        EEs() const;
    cuda_tool::CBufferView<Float>           EE_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> EE_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> EE_hessians() const;

    cuda_tool::CBufferView<Vector3i>        PEs() const;
    cuda_tool::CBufferView<Float>           PE_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> PE_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> PE_hessians() const;

    cuda_tool::CBufferView<Vector2i>        PPs() const;
    cuda_tool::CBufferView<Float>           PP_energies() const;
    cuda_tool::CDoubletVectorView<Float, 3> PP_gradients() const;
    cuda_tool::CTripletMatrixView<Float, 3> PP_hessians() const;

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
