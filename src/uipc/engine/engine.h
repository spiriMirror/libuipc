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
    Json to_json() const;

  protected:
    virtual void do_init(backend::WorldVisitor v) = 0;
    virtual void do_advance()                     = 0;
    virtual void do_sync()                        = 0;
    virtual void do_retrieve()                    = 0;
    virtual Json do_to_json() const;
};
}  // namespace uipc::engine
