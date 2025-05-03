// Don't put #pragma once here, this file may be included multiple times.

/*****************************************************************/ /**
 * @file   backend_name.h
 * @brief  This file records all the built-in backend names in the libuipc specification.
 * 
 * Programmers can define their own UIPC_BUILTIN_BACKEND macro outside this file, and include this file to get the built-in backend names.
 * 
 * @code{.cpp}
 * #define UIPC_BUILTIN_BACKEND(name) constexpr std::string_view name = #name
 * #include <pyuipc/builtin/backend_name.h>
 * #undef UIPC_BUILTIN_BACKEND
 * @endcode
 * 
 * @author MuGdxy
 * @date   September 2024
 *********************************************************************/

#ifdef UIPC_BUILTIN_BACKEND
UIPC_BUILTIN_BACKEND(none);
UIPC_BUILTIN_BACKEND(cuda);
#endif