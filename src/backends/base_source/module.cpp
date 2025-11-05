#include <backends/common/module.h>
#include <memory_resource>
#include <uipc/common/log.h>
#include <uipc/backend/module_init_info.h>

void uipc_init_module(UIPCModuleInitInfo* info)
{
    auto old_resource = std::pmr::get_default_resource();
    std::pmr::set_default_resource(info->memory_resource);
    uipc::logger::info("Synchronize backend module [{}]'s Polymorphic Memory Resource: {}->{}",
                 info->module_name,
                 (void*)old_resource,
                 (void*)std::pmr::get_default_resource());
}