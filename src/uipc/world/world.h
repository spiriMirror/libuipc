#pragma once
#include <uipc/world/scene.h>
#include <uipc/engine/i_engine.h>

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
    bool dump();
    bool recover();

    SizeT     frame() const;
  private:
    Scene*           m_scene  = nullptr;
    engine::IEngine* m_engine = nullptr;
    bool             m_valid  = true;
    void             sanity_check(Scene& s);
};
}  // namespace uipc::world
