#pragma once
#include <memory>
#include <memory_resource>
#include <uipc/common/allocator.h>

namespace uipc
{
template <typename T>
struct PmrDeleter
{
    using Allocator = uipc::Allocator<T>;
    void operator()(T* ptr) const;
};

template <typename T>
using U = std::unique_ptr<T, PmrDeleter<T>>;

template <typename T, typename... Args>
U<T> make_unique(Args&&... args);

template <typename DstT, typename SrcT>
U<DstT> static_pointer_cast(U<SrcT>&& src);

template <typename T>
using S = std::shared_ptr<T>;

template <typename T, typename... Args>
S<T> make_shared(Args&&... args);

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

    T* operator->();
    T& operator*();
    T* operator->() const;
    T& operator*() const;
    operator bool() const;
};
}  // namespace uipc

#include "details/smart_pointer.inl"