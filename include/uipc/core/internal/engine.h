#pragma once
#include <string>
#include <uipc/common/dllexport.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/core/i_engine.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/common/exception.h>

namespace uipc::core::internal
{
class World;

class UIPC_CORE_API Engine final : public std::enable_shared_from_this<Engine>
{
    class Impl;

  public:
    Engine(std::string_view backend_name,
           std::string_view workspace = "./",
           const Json&      config    = default_config());

    Engine(std::string_view backend_name,
           S<IEngine>       overrider,
           std::string_view workspace = "./",
           const Json&      config    = default_config());

    ~Engine();

    std::string_view         backend_name() const noexcept;
    std::string_view         workspace() const noexcept;
    EngineStatusCollection&  status();
    const FeatureCollection& features();

    Json to_json() const;

    static Json default_config();

  private:
    friend class internal::World;
    // only be called by internal::world
    void  init(internal::World& world);
    void  advance();
    void  sync();
    void  retrieve();
    bool  dump();
    bool  recover(SizeT dst_frame);
    SizeT frame() const;

    U<Impl> m_impl;
};
}  // namespace uipc::core::internal
