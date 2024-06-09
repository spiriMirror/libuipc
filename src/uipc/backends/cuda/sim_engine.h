#pragma once
#include <muda/ext/eigen/eigen_cxx20.h>
#include <uipc/backend/macro.h>
#include <uipc/engine/engine.h>
#include <sstream>
#include <uipc/backends/cuda/sim_system_collection.h>

namespace uipc::backend::cuda
{
class SimSystem;
class SimSystemCollection;

class UIPC_BACKEND_API SimEngine : public engine::IEngine
{
    class DeviceCommon;
    friend class SimSystem;

  public:
    SimEngine();
    ~SimEngine();

    // delete copy
    SimEngine(const SimEngine&)            = delete;
    SimEngine& operator=(const SimEngine&) = delete;

    DeviceCommon& device_common() noexcept;
    WorldVisitor& world() noexcept;

    template <std::derived_from<SimSystem> T>
    T* find();

  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;

  private:
    U<DeviceCommon>     m_device_common;
    std::stringstream   m_string_stream;
    U<WorldVisitor>     m_world_visitor;
    SimSystemCollection m_system_collection;
};
}  // namespace uipc::backend::cuda

#include "details/sim_engine.inl"