#pragma once
#include <sim_system.h>
#include <muda/buffer.h>
#include <utils/dump_utils.h>

namespace uipc::backend::cuda
{
class ABDBDF2State final : public SimSystem
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

        // q^{n-1}
        muda::DeviceBuffer<Vector12> q_n_1s;
        BufferDump                   dump_q_n_1s;
        // \dot{q}^{n-1}
        muda::DeviceBuffer<Vector12> q_v_n_1s;
        BufferDump                   dump_q_v_n_1s;
    };

    muda::CBufferView<Vector12> q_n_1s() const { return m_impl.q_n_1s.view(); }
    muda::CBufferView<Vector12> q_v_n_1s() const { return m_impl.q_v_n_1s.view(); }

  private:
    void resize(SizeT size);

    bool do_dump(DumpInfo& info) override;
    bool do_try_recover(RecoverInfo& info) override;
    void do_apply_recover(RecoverInfo& info) override;
    void do_clear_recover(RecoverInfo& info) override;

    friend class ABDBDF2Integrator;

    muda::BufferView<Vector12> q_n_1s() { return m_impl.q_n_1s.view(); }
    muda::BufferView<Vector12> q_v_n_1s() { return m_impl.q_v_n_1s.view(); }

    Impl m_impl;

    void do_build() override;
};
}  // namespace uipc::backend::cuda
