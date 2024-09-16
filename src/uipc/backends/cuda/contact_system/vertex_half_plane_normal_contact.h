#pragma once
#include <contact_system/contact_reporter.h>
#include <line_search/line_searcher.h>
#include <contact_system/contact_coeff.h>

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
        muda::CBufferView<IndexT>         contact_element_ids() const;
        Float                             d_hat() const;
        Float                             dt() const;
        Float                             eps_velocity() const;

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

        muda::BufferView<Vector3>   gradients() const noexcept;
        muda::BufferView<Matrix3x3> hessians() const noexcept;

      private:
        friend class VertexHalfPlaneNormalContact;

        muda::BufferView<Vector3>   m_gradients;
        muda::BufferView<Matrix3x3> m_hessians;
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
        void assemble(GlobalContactManager::ContactInfo& info);

        GlobalTrajectoryFilter* global_trajectory_filter      = nullptr;
        GlobalContactManager*   global_contact_manager = nullptr;
        GlobalVertexManager*    global_vertex_manager  = nullptr;

        SimSystemSlot<VertexHalfPlaneTrajectoryFilter> veretx_half_plane_trajectory_filter;

        SizeT PH_count = 0;
        Float dt;

        muda::DeviceBuffer<IndexT>    Ps;
        muda::DeviceBuffer<Float>     energies;
        muda::DeviceBuffer<Vector3>   gradients;
        muda::DeviceBuffer<Matrix3x3> m_hessians;

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
    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) override final;
    virtual void do_report_extent(GlobalContactManager::ContactExtentInfo& info) override final;
    virtual void do_assemble(GlobalContactManager::ContactInfo& info) override final;
    virtual void do_build() override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
