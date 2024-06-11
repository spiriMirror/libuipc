#pragma once
#include <memory>
#include <memory_resource>

namespace uipc
{
template <typename T, typename Dx = std::default_delete<T>>
using U = std::unique_ptr<T, Dx>;

template <typename T, typename Dx = std::default_delete<T>>
using S = std::shared_ptr<T>;

template <typename T, typename Dx = std::default_delete<T>>
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

    T* operator->();
    T& operator*();
    T* operator->() const;
    T& operator*() const;
    operator bool() const;
};
}  // namespace uipc
#include "details/smart_pointer.inl"