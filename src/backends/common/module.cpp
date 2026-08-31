#include <backends/common/module.h>
#include <memory_resource>
#include <uipc/common/log.h>
#include <uipc/backend/module_init_info.h>

#if !defined(UIPC_VERSION_MAJOR) || !defined(UIPC_VERSION_MINOR) || !defined(UIPC_VERSION_PATCH)
#error "Backend modules require the UIPC_VERSION_* compile definitions"
#endif

int uipc_query_module(UIPCBackendModuleInfo* info)
{
    if(info == nullptr || info->struct_size < sizeof(UIPCBackendModuleInfo))
        return 0;

    info->struct_size        = sizeof(UIPCBackendModuleInfo);
    info->abi_version        = UIPC_BACKEND_ABI_VERSION;
    info->uipc_version_major = UIPC_VERSION_MAJOR;
    info->uipc_version_minor = UIPC_VERSION_MINOR;
    info->uipc_version_patch = UIPC_VERSION_PATCH;
    info->backend_name       = UIPC_BACKEND_NAME;
    return 1;
}

void uipc_init_module(UIPCModuleInitInfo* info)
{
    auto old_resource = std::pmr::get_default_resource();
    std::pmr::set_default_resource(info->memory_resource);
    uipc::logger::info("Synchronize backend module [{}]'s Polymorphic Memory Resource: {}->{}",
                       info->module_name,
                       (void*)old_resource,
                       (void*)std::pmr::get_default_resource());
}
