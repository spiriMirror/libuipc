#pragma once

// Cross Platform Macros
#if defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
#define MUDA_PLATFORM_WIN64
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#define MUDA_PLATFORM_WIN32
#elif defined(__linux__)
#define MUDA_PLATFORM_LINUX
#elif defined(__APPLE__)
#define MUDA_PLATFORM_APPLE
#endif

// Windows 64 or 32
#if defined(MUDA_PLATFORM_WIN64) || defined(MUDA_PLATFORM_WIN32)
#define MUDA_PLATFORM_WINDOWS
#endif

// Function Attributes
#if defined(MUDA_PLATFORM_WINDOWS)
#define MUDA_STDCALL __stdcall
#define MUDA_CDECL __cdecl
#define MUDA_FASTCALL __fastcall
#define MUDA_FUNCTION_SIG __FUNCSIG__
#elif defined(MUDA_PLATFORM_LINUX)
#define MUDA_STDCALL __attribute__((__stdcall__))
#define MUDA_CDECL __attribute__((__cdecl__))
#define MUDA_FASTCALL __attribute__((__fastcall__))
#define MUDA_FUNCTION_SIG __PRETTY_FUNCTION__
#endif

#if defined(_MSVC_LANG)
#define MUDA_HAS_CXX20 _HAS_CXX20
#else
#define MUDA_HAS_CXX20 __cplusplus >= 202002L
#endif
