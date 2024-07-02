#pragma once
#include <contact_system/contact_reporter.h>

namespace uipc::backend::cuda
{
class SimplexContactConstitution : public ContactReporter
{
  public:
    using ContactReporter::ContactReporter;

    class Impl;

    class PrimitiveCountInfo
    {
      public:
        PrimitiveCountInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        void count(SizeT PT_count, SizeT EE_count, SizeT PE_count, SizeT PP_count) noexcept
        {
            m_impl->PT_count = PT_count;
            m_impl->EE_count = EE_count;
            m_impl->PE_count = PE_count;
            m_impl->PP_count = PP_count;
        }

      private:
        Impl* m_impl;
    };

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


    class Impl
    {
      public:
        void prepare();
        void assemble(GlobalContactManager::ContactInfo& info);

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
    };

  protected:
    virtual void do_build() override;
    virtual void do_report_count(PrimitiveCountInfo& info) = 0;
    virtual void do_assemble(ContactInfo& info)            = 0;

  private:
    virtual void do_report_extent(GlobalContactManager::ContactExtentInfo& info) override final;
    virtual void do_assemble(GlobalContactManager::ContactInfo& info) override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
