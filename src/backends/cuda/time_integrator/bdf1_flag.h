#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
/*
* @brief FEM BDF1 Flag
 *
 * This is a flag class, by `require<BDF1Flag>()` you can easily shutdown you dependent 
 * systems if the BDF1 integration method is not enabled.
*/
class BDF1Flag final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    void do_build() override;
};
}  // namespace uipc::backend::cuda