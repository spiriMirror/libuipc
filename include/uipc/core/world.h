#pragma once
#include <uipc/core/scene.h>
#include <uipc/core/feature_collection.h>

namespace uipc::backend
{
class WorldVisitor;
}

namespace uipc::core::internal
{
class World;
}

namespace uipc::core
{
class Engine;

class UIPC_CORE_API World final
{
    friend class backend::WorldVisitor;
    friend class SanityChecker;

  public:
    World(Engine& e) noexcept;
    ~World();

    World(const World&)            = delete;
    World(World&&)                 = default;
    World& operator=(const World&) = delete;
    World& operator=(World&&)      = default;

    void init(Scene& s);

    void advance();
    void sync();
    void retrieve();
    bool dump();
    bool recover(SizeT aim_frame = ~0ull);
    bool is_valid() const;

    SizeT frame() const;

    const FeatureCollection& features() const;

    // Allow create a core::World from a core::internal::World
    World(S<internal::World> w) noexcept;

  private:
    S<internal::World> m_internal;
};
}  // namespace uipc::core
