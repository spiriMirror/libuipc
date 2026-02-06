#pragma once
#ifdef __cplusplus
#define RBC_EXTERN_C extern "C"
#define RBC_NOEXCEPT noexcept
#else
#define RBC_EXTERN_C
#define RBC_NOEXCEPT
#endif

#ifdef _WIN32
#define RBC_DECLSPEC_DLL_EXPORT __declspec(dllexport)
#define RBC_DECLSPEC_DLL_IMPORT __declspec(dllimport)
#else
#define RBC_DECLSPEC_DLL_EXPORT __attribute__((visibility("default")))
#define RBC_DECLSPEC_DLL_IMPORT
#endif

#define RBC_UIPC_API RBC_EXTERN_C RBC_DECLSPEC_DLL_EXPORT