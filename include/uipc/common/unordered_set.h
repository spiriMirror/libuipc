#pragma once
#include <memory_resource>
#include <unordered_set>

namespace uipc
{
/**
 * @brief uipc uses std::pmr::unordered_map as the default unordered_map type.
 */
using std::pmr::unordered_set;
}  // namespace uipc
