#include <pipeline/ipc_pipeline_flag.h>
#include <sim_engine.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::IPCPipelineFlag>
{
  public:
    static U<cuda::IPCPipelineFlag> create(SimEngine& engine)
    {
        auto scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto ctype_attr = scene.config().find<std::string>("contact/constitution");

        if(ctype_attr->view()[0] != "ipc")
        {
            return nullptr;  // Not an IPC pipeline
        }
        return uipc::make_unique<cuda::IPCPipelineFlag>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCPipelineFlag);
void IPCPipelineFlag::do_build() {}
}  // namespace uipc::backend::cuda
