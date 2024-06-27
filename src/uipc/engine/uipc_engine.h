#pragma once
#include <string>
#include <uipc/common/macro.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/engine/engine.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/common/exception.h>
namespace uipc::world
{
class World;
}

namespace uipc::engine
{
class UIPC_CORE_API UIPCEngine : public IEngine
{
    class Impl;

  public:
    UIPCEngine(std::string_view backend_name, std::string_view workspace = "./");
    ~UIPCEngine();

  protected:
    virtual void do_init(backend::WorldVisitor v) override;
    virtual void do_advance() override;
    virtual void do_sync() override;
    virtual void do_retrieve() override;
    virtual Json do_to_json() const override;

  private:
    U<Impl> m_impl;
};

class UIPC_CORE_API EngineException : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::engine
