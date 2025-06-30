#include <time_integrator/bdf1_flag.h>
#include <sim_engine.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::BDF1Flag>
{
  public:
    static U<cuda::BDF1Flag> create(SimEngine& engine)
    {
        auto scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        if(scene.info()["integrator"]["type"] != "bdf1")
        {
            return nullptr;  // Not a BDF1 integrator
        }
        return uipc::make_unique<cuda::BDF1Flag>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(BDF1Flag);
void BDF1Flag::do_build() {}
}  // namespace uipc::backend::cuda