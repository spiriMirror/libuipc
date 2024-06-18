#pragma once
#include <type_define.h>

#include <i_sim_system.h>
#include <sim_action.h>
#include <sim_system_auto_register.h>
#include <string_view>
#include <uipc/backend/visitors/world_visitor.h>

namespace uipc::backend::cuda
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

    virtual void build() override{};

    /**
     * @brief register an action to be executed when the scene is initialized
     * 
     * This function can only be called in build() function
     */
    void          on_init_scene(std::function<void()>&& action);

    WorldVisitor& world();
  private:
    SimEngine&           m_sim_engine;
    SimSystemCollection& collection();
};
}  // namespace uipc::backend::cuda

#include "details/sim_system.inl"