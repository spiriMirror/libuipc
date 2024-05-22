#pragma once
#include <memory>

namespace uipc
{
template <typename T>
using U = std::unique_ptr<T>;

template <typename T>
using S = std::shared_ptr<T>;

template <typename T>
using W = std::weak_ptr<T>;
}  // namespace uipc
