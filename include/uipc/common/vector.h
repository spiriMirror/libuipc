#pragma once
#include <memory_resource>
#include <vector>

namespace uipc
{
/**
 * @brief uipc uses std::pmr::vector as the default vector type.
 */
using std::pmr::vector;
//using std::vector;
}  // namespace uipc
