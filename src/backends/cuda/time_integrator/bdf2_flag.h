#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
/*
* @brief FEM/ABD BDF2 Flag
*
* This is a flag class, by `require<BDF2Flag>()` you can easily shutdown your dependent
* systems if the BDF2 integration method is not enabled.
*/
class BDF2Flag final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    void do_build() override;
};
}  // namespace uipc::backend::cuda
