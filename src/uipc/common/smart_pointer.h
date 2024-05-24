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

/**
 * @brief A different version of weak pointer that allows for easier access to the underlying object
 */
template <typename T>
class P : public std::weak_ptr<T>
{
    using Base = std::weak_ptr<T>;

  public:
    using Base::Base;
    using Base::operator=;

    T* operator->() const;
    T& operator*() const;
    operator bool() const;
};
}  // namespace uipc
#include "details/smart_pointer.inl"