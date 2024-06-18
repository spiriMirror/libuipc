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
    class DeviceCommon;
    friend class SimSystem;

  public:
    SimEngine();
    ~SimEngine();

    // delete copy_from
    SimEngine(const SimEngine&)            = delete;
    SimEngine& operator=(const SimEngine&) = delete;

    WorldVisitor& world() noexcept;

  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;

  private:
    DeviceCommon&       device_common() noexcept;
    U<DeviceCommon>     m_device_common;
    std::stringstream   m_string_stream;
    U<WorldVisitor>     m_world_visitor;
    SimSystemCollection m_system_collection;
    list<SimAction>     m_on_init_scene;
    SimEngineState      m_state = SimEngineState::None;

    // Events
    void event_init_scene();

    template <std::derived_from<ISimSystem> T>
    T* find();
};
}  // namespace uipc::backend::cuda

#include "details/sim_engine.inl"