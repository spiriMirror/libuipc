#pragma once
#include <uipc/backend/macro.h>
#include <uipc/backends/common/sim_engine.h>

namespace uipc::backend::none
{
class NoneSimSystem;
class UIPC_BACKEND_API NoneSimEngine : public SimEngine
{
  public:
    NoneSimEngine();
    ~NoneSimEngine();

  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;

  private:
    NoneSimSystem*              m_system = nullptr;
};
}  // namespace uipc::backend::none
