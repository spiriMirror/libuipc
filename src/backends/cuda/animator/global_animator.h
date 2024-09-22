#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class Animator;
class GlobalAnimator final : public SimSystem
{
  public:
    using SimSystem::SimSystem;


  private:
    friend class SimEngine;
    void init();  // only be called by SimEngine
    void step();  // only be called by SimEngine
    friend class Animator;
    void register_animator(Animator* animator);  // only be called by Animator


    virtual void do_build() override;

    SimSystemSlotCollection<Animator> m_animators;
};
}  // namespace uipc::backend::cuda
