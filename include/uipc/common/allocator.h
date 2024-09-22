#pragma once
#include <memory_resource>

namespace uipc
{
template <typename T>
using Allocator = std::pmr::polymorphic_allocator<T>;
}
