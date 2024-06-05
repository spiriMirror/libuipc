#pragma once
#include <uipc/common/macro.h>
#include <uipc/backend/visitors/world_visitor.h>

namespace uipc::world
{
class World;
}

namespace uipc::engine
{
class UIPC_CORE_API IEngine
{
  public:
    virtual ~IEngine() = default;
    void init(backend::WorldVisitor v);
    void advance();
    void sync();
    void retrieve();

  protected:
    virtual void do_init(backend::WorldVisitor v) = 0;
    virtual void do_advance()                     = 0;
    virtual void do_sync()                        = 0;
    virtual void do_retrieve()                    = 0;
};
}  // namespace uipc::engine
