#pragma once
#include <uipc/core/internal/scene.h>
#include <uipc/core/internal/engine.h>
#include <uipc/core/feature_collection.h>

namespace uipc::core::internal
{
class UIPC_CORE_API World final : public std::enable_shared_from_this<World>
{
    friend class backend::WorldVisitor;
    friend class SanityChecker;

  public:
    World(internal::Engine& e) noexcept;
    void init(internal::Scene& s);

    void advance();
    void sync();
    void retrieve();
    void backward();
    bool dump();
    bool recover(SizeT aim_frame = ~0ull);
    bool is_valid() const;

    SizeT frame() const;

    const FeatureCollection& features() const;

  private:
    S<internal::Scene> m_scene = nullptr;
    // MUST NOT be a shared_ptr to avoid circular reference
    W<internal::Engine> m_engine;
    bool                m_valid = true;
    void                sanity_check(Scene& s);
};
}  // namespace uipc::core::internal
