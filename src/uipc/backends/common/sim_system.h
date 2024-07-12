#pragma once
#include <uipc/backends/common/i_sim_system.h>
#include <string_view>
#include <uipc/backends/common/sim_system_auto_register.h>
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
    template <std::derived_from<SimSystem> T>
    T* find();

    template <std::derived_from<SimSystem> T>
    T& require();

    virtual Json do_to_json() const override;
    SimEngine&   engine() noexcept;

  private:
    SimEngine& m_sim_engine;
    bool       m_engine_aware = false;
    bool       m_valid        = true;
    bool       m_is_building  = false;

    vector<ISimSystem*>  m_dependencies;
    SimSystemCollection& collection() noexcept;
    void                 set_building(bool b) noexcept override;
    virtual bool         get_is_building() const noexcept override;



    virtual void set_engine_aware() noexcept final;
    virtual bool get_engine_aware() const noexcept final;

    virtual bool get_valid() const noexcept override final;
    virtual void set_invalid() noexcept override final;

    virtual span<ISimSystem* const> get_dependencies() const noexcept override final;

    SimSystem* find_system(SimSystem* ptr);
    SimSystem* require_system(SimSystem* ptr);

    void check_state(std::string_view function_name) noexcept;
};
}  // namespace uipc::backend

#include <uipc/backends/common/details/sim_system.inl>
