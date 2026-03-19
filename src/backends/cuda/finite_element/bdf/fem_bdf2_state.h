#pragma once
#include <sim_system.h>
#include <muda/buffer.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class FEMBDF2State final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        // x^{n-1}
        muda::DeviceBuffer<Vector3> x_n_1s;
        BufferDump                  dump_x_n_1s;
        // \dot{x}^{n-1}
        muda::DeviceBuffer<Vector3> v_n_1s;
        BufferDump                  dump_v_n_1s;
    };

    muda::CBufferView<Vector3> x_n_1s() const { return m_impl.x_n_1s.view(); }
    muda::CBufferView<Vector3> v_n_1s() const { return m_impl.v_n_1s.view(); }

  private:
    void resize(SizeT size);

    bool do_dump(DumpInfo& info) override;
    bool do_try_recover(RecoverInfo& info) override;
    void do_apply_recover(RecoverInfo& info) override;
    void do_clear_recover(RecoverInfo& info) override;

    friend class FEMBDF2Integrator;

    muda::BufferView<Vector3> x_n_1s() { return m_impl.x_n_1s.view(); }
    muda::BufferView<Vector3> v_n_1s() { return m_impl.v_n_1s.view(); }

    Impl m_impl;

    void do_build() override;
};
}  // namespace uipc::backend::cuda
