#pragma once
#ifndef MUDA_CHECK_ON
#define MUDA_CHECK_ON 0
#endif
#ifndef MUDA_COMPUTE_GRAPH_ON
#define MUDA_COMPUTE_GRAPH_ON 0
#endif

namespace muda
{
constexpr bool RUNTIME_CHECK_ON = MUDA_CHECK_ON;
constexpr bool COMPUTE_GRAPH_ON = MUDA_COMPUTE_GRAPH_ON;
namespace config
{
    constexpr bool on(bool cond = false)
    {
        return cond && RUNTIME_CHECK_ON;
    }
}  // namespace config
// debug viewer
constexpr bool DEBUG_VIEWER = config::on(true);
// trap on error happens
constexpr bool TRAP_ON_ERROR = config::on(true);
// light workload block size
constexpr int LIGHT_WORKLOAD_BLOCK_SIZE = 256;
// middle workload block size
constexpr int MIDDLE_WORKLOAD_BLOCK_SIZE = 128;
// heavy workload block size
constexpr int HEAVY_WORKLOAD_BLOCK_SIZE = 64;
}  // namespace muda
