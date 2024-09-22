#pragma once

namespace uipc
{
#if UIPC_RUNTIME_CHECK
constexpr bool RUNTIME_CHECK = true;
#else
constexpr bool RUNTIME_CHECK = false;
#endif
}  // namespace uipc
