#pragma once
#include <memory_resource>
#include <list>

namespace uipc
{
/**
 * @brief uipc uses std::pmr::list as the default list type.
 */
using std::pmr::list;
}  // namespace uipc
