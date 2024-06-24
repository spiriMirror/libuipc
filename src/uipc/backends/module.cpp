#include <uipc/backends/module.h>
#include <memory_resource>
#include <uipc/common/log.h>

void uipc_init_module(UIPCModuleInitInfo* info)
{
    std::pmr::set_default_resource(info->memory_resource);
    spdlog::info("UIPC module memory resource: 0x{}", (void*)std::pmr::get_default_resource());
}
