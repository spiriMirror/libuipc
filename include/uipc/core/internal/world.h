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
    internal::Scene*  m_scene  = nullptr;
    internal::Engine* m_engine = nullptr;
    bool              m_valid  = true;
    void              sanity_check(Scene& s);
};
}  // namespace uipc::core::internal
