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

    class Impl;

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        muda::CBuffer2DView<ContactCoeff> contact_tabular() const;
        muda::CBufferView<Vector4i>       friction_PTs() const;
        muda::CBufferView<Vector4i>       friction_EEs() const;
        muda::CBufferView<Vector3i>       friction_PEs() const;
        muda::CBufferView<Vector2i>       friction_PPs() const;
        muda::CBufferView<Vector3>        positions() const;
        muda::CBufferView<Vector3>        prev_positions() const;
        muda::CBufferView<Vector3>        rest_positions() const;
        muda::CBufferView<Float>          thicknesses() const;
        muda::CBufferView<IndexT>         contact_element_ids() const;
        Float                             d_hat() const;
        Float                             dt() const;
        Float                             eps_velocity() const;

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

      private:
        friend class SimplexFrictionalContact;
        muda::DoubletVectorView<Float, 3> m_PT_gradients;
        muda::TripletMatrixView<Float, 3> m_PT_hessians;

        muda::DoubletVectorView<Float, 3> m_EE_gradients;
        muda::TripletMatrixView<Float, 3> m_EE_hessians;

        muda::DoubletVectorView<Float, 3> m_PE_gradients;
        muda::TripletMatrixView<Float, 3> m_PE_hessians;

        muda::DoubletVectorView<Float, 3> m_PP_gradients;
        muda::TripletMatrixView<Float, 3> m_PP_hessians;
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

        muda::BufferView<Float> friction_PT_energies() const noexcept
        {
            return m_PT_energies;
        }
        muda::BufferView<Float> friction_EE_energies() const noexcept
        {
            return m_EE_energies;
        }
        muda::BufferView<Float> friction_PE_energies() const noexcept
        {
            return m_PE_energies;
        }
        muda::BufferView<Float> friction_PP_energies() const noexcept
        {
            return m_PP_energies;
        }

      private:
        friend class SimplexFrictionalContact;
        muda::BufferView<Float> m_PT_energies;
        muda::BufferView<Float> m_EE_energies;
        muda::BufferView<Float> m_PE_energies;
        muda::BufferView<Float> m_PP_energies;
    };

    class Impl
    {
      public:
        void compute_energy(SimplexFrictionalContact*         contact,
                            GlobalContactManager::EnergyInfo& info);

        GlobalTrajectoryFilter* global_trajectory_filter = nullptr;
        GlobalContactManager*   global_contact_manager   = nullptr;
        GlobalVertexManager*    global_vertex_manager    = nullptr;

        SimSystemSlot<SimplexTrajectoryFilter> simplex_trajectory_filter;

        SizeT PT_count = 0;
        SizeT EE_count = 0;
        SizeT PE_count = 0;
        SizeT PP_count = 0;
        Float dt       = 0;

        muda::DeviceBuffer<Float> energies;

        Float reserve_ratio = 1.1;

        template <typename T>
        void loose_resize(muda::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve(size * reserve_ratio);
            }
            buffer.resize(size);
        }
    };

  protected:
    virtual void do_build(BuildInfo& info)           = 0;
    virtual void do_compute_energy(EnergyInfo& info) = 0;
    virtual void do_assemble(ContactInfo& info)      = 0;

  private:
    virtual void do_build(ContactReporter::BuildInfo& info) override final;
    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) override final;
    virtual void do_report_extent(GlobalContactManager::ContactExtentInfo& info) override final;
    virtual void do_assemble(GlobalContactManager::ContactInfo& info) override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
