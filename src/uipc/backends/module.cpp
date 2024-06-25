#include <uipc/backends/module.h>
#include <memory_resource>
#include <uipc/common/log.h>

void uipc_init_module(UIPCModuleInitInfo* info)
{
    auto old_resource = std::pmr::get_default_resource();
    std::pmr::set_default_resource(info->memory_resource);
    spdlog::info("Synchronize backend module [{}] memory resource: {}->{}",
                 info->module_name,
                 (void*)old_resource,
                 (void*)std::pmr::get_default_resource());
}
