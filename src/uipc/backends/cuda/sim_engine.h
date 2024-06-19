#pragma once
#include <type_define.h>
#include <uipc/backend/macro.h>
#include <uipc/engine/engine.h>
#include <sstream>
#include <sim_system_collection.h>
#include <sim_action.h>
#include <sim_engine_state.h>

namespace uipc::backend::cuda
{
class UIPC_BACKEND_API SimEngine : public engine::IEngine
{
    class DeviceImpl;
    friend class SimSystem;

  public:
    SimEngine();
    ~SimEngine();

    // delete copy_from
    SimEngine(const SimEngine&)            = delete;
    SimEngine& operator=(const SimEngine&) = delete;

    WorldVisitor&  world() noexcept;
    SimEngineState state() const noexcept;

  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;

  private:
    DeviceImpl&         device_impl() noexcept;
    U<DeviceImpl>       m_device_impl;
    std::stringstream   m_string_stream;
    U<WorldVisitor>     m_world_visitor;
    SimSystemCollection m_system_collection;
    SimEngineState      m_state = SimEngineState::None;

    // Events
    list<SimAction> m_on_init_scene;
    void            event_init_scene();
    list<SimAction> m_on_rebuild_scene;
    void            event_rebuild_scene();
    list<SimAction> m_on_write_scene;
    void            event_write_scene();

    template <std::derived_from<ISimSystem> T>
    T* find();

    void register_all_systems();  // called in do_init() only.
};
}  // namespace uipc::backend::cuda

#include "details/sim_engine.inl"