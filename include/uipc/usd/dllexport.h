#pragma once

#ifdef _WIN32
#ifdef UIPC_USD_EXPORT_DLL
#define UIPC_USD_API __declspec(dllexport)
#else
#define UIPC_USD_API __declspec(dllimport)
#endif
#else
#define UIPC_USD_API
#endif
