#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/core/sanity_checker.h>
#include <uipc/backend/module_init_info.h>

using SanityCheckerCollectionInterface = uipc::core::ISanityCheckerCollection;
using SanityCheckerCollectionCreateInfo = uipc::core::SanityCheckerCollectionCreateInfo;


UIPC_EXPORT_API void uipc_init_module(UIPCModuleInitInfo* info);

UIPC_EXPORT_API SanityCheckerCollectionInterface* uipc_create_sanity_checker_collection(
    SanityCheckerCollectionCreateInfo* info);

UIPC_EXPORT_API void uipc_destroy_sanity_checker_collection(SanityCheckerCollectionInterface* collection);
