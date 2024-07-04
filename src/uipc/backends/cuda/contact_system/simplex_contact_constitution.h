#pragma once
#include <contact_system/contact_reporter.h>
#include <line_search/line_searcher.h>
#include <contact_system/contact_coeff.h>
namespace uipc::backend::cuda
{
class SimplexDCDFilter;
class SimplexContactConstitution : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;

    class Impl;

    class ContactInfo
    {
      public:
        ContactInfo(Impl* impl)
            : m_impl(impl)
        {
        }


      private:
        Impl* m_impl;
    };

    class BuildInfo
    {
      public:
    };

    class EnergyInfo
    {
      public:
        EnergyInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        muda::CBuffer2DView<ContactCoeff> contact_tabular() const;
        muda::CBufferView<Vector4i>       PTs() const;
        muda::CBufferView<Vector4i>       EEs() const;
        muda::CBufferView<Vector3i>       PEs() const;
        muda::CBufferView<Vector2i>       PPs() const;
        muda::CBufferView<Vector3>        positions() const;
        Float                             d_hat() const;

        muda::BufferView<Float> PT_energies() noexcept { return m_PT_energies; }
        muda::BufferView<Float> EE_energies() noexcept { return m_EE_energies; }
        muda::BufferView<Float> PE_energies() noexcept { return m_PE_energies; }
        muda::BufferView<Float> PP_energies() noexcept { return m_PP_energies; }

      private:
        friend class SimplexContactConstitution;
        Impl*                   m_impl;
        muda::BufferView<Float> m_PT_energies;
        muda::BufferView<Float> m_EE_energies;
        muda::BufferView<Float> m_PE_energies;
        muda::BufferView<Float> m_PP_energies;
    };

    class Impl
    {
      public:
        void prepare();
        void assemble(GlobalContactManager::ContactInfo& info);

        SimplexDCDFilter*     simplex_dcd_filter     = nullptr;
        GlobalContactManager* global_contact_manager = nullptr;
        GlobalVertexManager*  global_vertex_manager  = nullptr;

        SizeT PT_count = 0;
        SizeT EE_count = 0;
        SizeT PE_count = 0;
        SizeT PP_count = 0;

        muda::DeviceBuffer<Vector4i>    PT_EE_indices;
        muda::DeviceBuffer<Matrix12x12> PT_EE_hessians;
        muda::DeviceBuffer<Vector12>    PT_EE_gradients;

        muda::DeviceBuffer<Vector3i>  PE_indices;
        muda::DeviceBuffer<Matrix9x9> PE_hessians;
        muda::DeviceBuffer<Vector9>   PE_gradients;

        muda::DeviceBuffer<Vector2i>  PP_indices;
        muda::DeviceBuffer<Matrix6x6> PP_hessians;
        muda::DeviceBuffer<Vector6>   PP_gradients;

        muda::DeviceBuffer<Float> energies;
    };

  protected:
    virtual void do_build(BuildInfo& info)           = 0;
    virtual void do_compute_energy(EnergyInfo& info) = 0;
    virtual void do_assemble(ContactInfo& info)      = 0;

  private:
    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) override final;
    virtual void do_report_extent(GlobalContactManager::ContactExtentInfo& info) override final;
    virtual void do_assemble(GlobalContactManager::ContactInfo& info) override final;
    virtual void do_build() override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
