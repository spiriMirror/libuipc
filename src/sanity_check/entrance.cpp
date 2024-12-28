#include <entrance.h>
#include <sanity_checker_collection.h>
#include <memory_resource>

void uipc_init_module(UIPCModuleInitInfo* info)
{
    auto old_resource = std::pmr::get_default_resource();
    std::pmr::set_default_resource(info->memory_resource);
    spdlog::info("Synchronize module [{}]'s Polymorphic Memory Resource: {}->{}",
                 info->module_name,
                 (void*)old_resource,
                 (void*)std::pmr::get_default_resource());
}

SanityCheckerCollectionInterface* uipc_create_sanity_checker_collection(SanityCheckerCollectionCreateInfo* info)
{
    return new uipc::sanity_check::SanityCheckerCollection(info->workspace);
}

void uipc_destroy_sanity_checker_collection(SanityCheckerCollectionInterface* collection)
{
    delete collection;
}