#pragma once
#include <stack>
#include <deque>

namespace uipc
{
template <typename T>
using stack = std::stack<T, std::pmr::deque<T>>;
}
