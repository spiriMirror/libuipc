#pragma once
#include <backends/common/i_sim_system.h>
#include <uipc/common/macro.h>
#include <string_view>
#include <backends/common/sim_system_auto_register.h>
#include <uipc/backend/visitors/world_visitor.h>

namespace uipc::backend
{
class SimEngine;
class SimSystemCollection;

class SimSystem : public ISimSystem
{
    friend class SimEngine;

  public:
    SimSystem(SimEngine& sim_engine) noexcept;

  protected:
    using QueryOptions = SimSystemCollection::QueryOptions;

    template <std::derived_from<SimSystem> T>
    T* find(const QueryOptions& options = {});

    template <std::derived_from<SimSystem> T>
    T& require(const QueryOptions& options = {});

    virtual Json             do_to_json() const override;
    SimEngine&               engine() noexcept;
    std::string_view         workspace() const noexcept;
    core::FeatureCollection& features() noexcept;

  private:
    SimEngine&          m_sim_engine;
    bool                m_engine_aware = false;
    bool                m_valid        = true;
    bool                m_is_building  = false;
    mutable std::string m_name;

    vector<ISimSystem*> m_strong_dependencies;
    vector<ISimSystem*> m_weak_dependencies;

    SimSystemCollection& collection() noexcept;
    virtual void         set_building(bool b) noexcept override;
    virtual bool         get_is_building() const noexcept override;

    virtual std::string_view get_name() const noexcept override;

    virtual void set_engine_aware() noexcept final;
    virtual bool get_engine_aware() const noexcept final;

    virtual bool get_valid() const noexcept override final;
    virtual void set_invalid() noexcept override final;

    virtual span<ISimSystem* const> get_strong_dependencies() const noexcept override final;
    virtual span<ISimSystem* const> get_weak_dependencies() const noexcept override final;

    SimSystem* find_system(SimSystem* ptr);
    SimSystem* require_system(SimSystem* ptr);

    void check_state(std::string_view function_name) noexcept;
};
}  // namespace uipc::backend

#include <backends/common/details/sim_system.inl>

/**
 * @brief Register a SimSystem, which will be automatically created by the SimEngine.
 */
#define REGISTER_SIM_SYSTEM(SimSystem)                                                          \
    namespace auto_register                                                                     \
    {                                                                                           \
        static ::uipc::backend::SimSystemAutoRegister UIPC_NAME_WITH_ID(SimSystemAutoRegister)( \
            ::uipc::backend::detail::register_system_creator<SimSystem>());                     \
    }

// End of file, remove the warning: backslash-newline at end of file