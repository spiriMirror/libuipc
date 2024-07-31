#pragma once
#include <uipc/backend/macro.h>
#include <uipc/engine/engine.h>
#include <uipc/backends/common/sim_system_collection.h>
#include <uipc/backends/common/i_sim_system.h>

namespace uipc::backend
{
class UIPC_BACKEND_API SimEngine : public engine::IEngine
{
    class DeviceImpl;
    friend class SimSystem;

  public:
    SimEngine()  = default;
    ~SimEngine() = default;

    SimEngine(const SimEngine&)            = delete;
    SimEngine& operator=(const SimEngine&) = delete;

  protected:
    virtual Json do_to_json() const override;

    /**
     * @brief Build the SimSystems in the engine.
     * 
     * This function will check the dependencies of each SimSystem and check the validity of the SimSystems.
     * If some SimSystems are invalid, any SimSystems that depend on them will also be invalidated.
     */
    void build_systems();

    /**
     * @brief Dump the system information to "systems.json" in engine working directory.
     */
    virtual void dump_system_info() const;

    /**
     * @brief Find certain SimSystem
     * 
     * Find certain SimSystem by type T.
     * 1. If the SimSystem is not found, return nullptr.
     * 2. If the SimSystem is invalid, return nullptr.
     * 3. Else, return the SimSystem.
     * 
     */
    template <std::derived_from<ISimSystem> T>
    T* find();

    /**
     * @brief Require certain SimSystem
     * 
     * Require certain SimSystem by type T.
     * 1. If the SimSystem is not found, throw SimEngineException.
     * 2. If the SimSystem is invalid, throw SimEngineException.
     * 3. Else, return the SimSystem.
     * 
     */
    template <std::derived_from<ISimSystem> T>
    T& require();

    span<ISimSystem* const> systems() noexcept;

  protected:
    virtual bool do_dump() override;
    virtual bool do_recover() override;

  private:
    ISimSystem* find_system(ISimSystem* ptr);
    ISimSystem* require_system(ISimSystem* ptr);

    SimSystemCollection m_system_collection;
};

class SimEngineException : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::backend

#include "details/sim_engine.inl"
