#pragma once

#ifdef _WIN32
#ifdef UIPC_VDB_EXPORT_DLL
#define UIPC_VDB_API __declspec(dllexport)
#else
#define UIPC_VDB_API __declspec(dllimport)
#endif
#else
#define UIPC_VDB_API
#endif