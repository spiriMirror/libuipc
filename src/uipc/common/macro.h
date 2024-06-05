#pragma once

#ifdef __cplusplus
#define UIPC_EXTERN_C extern "C"
#define UIPC_NOEXCEPT noexcept
#else
#define UIPC_EXTERN_C
#define UIPC_NOEXCEPT
#endif

#ifdef _MSC_VER
#define UIPC_FORCE_INLINE inline
#define UIPC_NEVER_INLINE __declspec(noinline)
#define UIPC_EXPORT_API UIPC_EXTERN_C __declspec(dllexport)
#define UIPC_IMPORT_API UIPC_EXTERN_C __declspec(dllimport)
#else
#define UIPC_FORCE_INLINE __attribute__((always_inline, hot)) inline
#define UIPC_NEVER_INLINE __attribute__((noinline))
#define UIPC_EXPORT_API UIPC_EXTERN_C __attribute__((visibility("default")))
#define UIPC_IMPORT_API UIPC_EXTERN_C
#endif

#ifdef _MSC_VER

#ifdef UIPC_CORE_EXPORT_DLL
#define UIPC_CORE_API __declspec(dllexport)
#else
#define UIPC_CORE_API __declspec(dllimport)
#endif

#ifdef UIPC_BACKEND_EXPORT_DLL
#define UIPC_BACKEND_API __declspec(dllexport)
#else
#define UIPC_BACKEND_API __declspec(dllimport)
#endif

#else
#define UIPC_CORE_API
#define UIPC_BACKEND_API
#endif
