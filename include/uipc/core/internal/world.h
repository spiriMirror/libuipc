#pragma once
#include <uipc/core/sanity_checker.h>
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
    bool dump();
    bool recover(SizeT aim_frame = ~0ull);
    bool is_valid() const;

    SizeT frame() const;

    const FeatureCollection& features() const;

    SanityChecker&       sanity_checker();
    const SanityChecker& sanity_checker() const;


  private:
    S<internal::Scene> m_scene = nullptr;
    // MUST NOT be a shared_ptr to avoid circular reference
    W<internal::Engine> m_engine;
    S<SanityChecker>    m_sanity_checker;
    bool                m_valid = true;
    void                _sanity_check();
};
}  // namespace uipc::core::internal
