#pragma once
#include <uipc/backend/macro.h>
#include <uipc/engine/engine.h>

namespace uipc::backend
{
class UIPC_BACKEND_API NoneEngine : public engine::IEngine
{
  public:
    NoneEngine();
    ~NoneEngine();

  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;
};
}  // namespace uipc::backend
