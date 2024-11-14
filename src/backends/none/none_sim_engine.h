#pragma once
#include <uipc/backend/macro.h>
#include <backends/common/sim_engine.h>

namespace uipc::backend::none
{
class NoneSimSystem;
class NoneSimEngine final : public SimEngine
{
  public:
    NoneSimEngine(EngineCreateInfo*);
    ~NoneSimEngine();

  private:
    void         do_init(InitInfo&) override;
    virtual bool do_dump(DumpInfo&) override;
    virtual void do_backward() override;
    virtual bool do_try_recover(RecoverInfo&) override;
    virtual void do_apply_recover(RecoverInfo&) override;
    virtual void do_clear_recover(RecoverInfo&) override;

    void  do_advance() override;
    void  do_sync() override;
    void  do_retrieve() override;
    SizeT get_frame() const override;

    NoneSimSystem* m_system = nullptr;
    SizeT          m_frame  = 0;
};
}  // namespace uipc::backend::none
