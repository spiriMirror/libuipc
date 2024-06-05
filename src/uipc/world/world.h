#pragma once
#include <uipc/world/scene.h>
#include <uipc/engine/engine.h>
namespace uipc::backend
{
class WorldVisitor;
}

namespace uipc::world
{
class UIPC_CORE_API World
{
    friend class backend::WorldVisitor;

  public:
    World(engine::IEngine& e) noexcept;
    void init(Scene& s);

    void advance();
    void sync();
    void retrieve();

  private:
    Scene*           m_scene  = nullptr;
    engine::IEngine* m_engine = nullptr;
};
}  // namespace uipc::world
