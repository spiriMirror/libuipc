#include <uipc/backends/common/module.h>
#include <memory_resource>
#include <uipc/common/log.h>

void uipc_init_module(UIPCModuleInitInfo* info)
{
    auto old_resource = std::pmr::get_default_resource();
    std::pmr::set_default_resource(info->memory_resource);
    spdlog::info("Synchronize backend module [{}]'s Polymorphic Memory Resource: {}->{}",
                 info->module_name,
                 (void*)old_resource,
                 (void*)std::pmr::get_default_resource());

    uipc::backend::ModuleInfo::instance().init(*info);
}

namespace uipc::backend
{
void ModuleInfo::init(const UIPCModuleInitInfo& info) noexcept
{
    m_workspace = info.module_workspace;
}

ModuleInfo& ModuleInfo::instance() noexcept
{
    static ModuleInfo instance;
    return instance;
}
}  // namespace uipc::backend