#pragma once

#ifdef _MSC_VER
#ifdef UIPC_BACKEND_EXPORT_DLL
#define UIPC_BACKEND_API __declspec(dllexport)
#else
#define UIPC_BACKEND_API __declspec(dllimport)
#endif

#else
#define UIPC_BACKEND_API
#endif
