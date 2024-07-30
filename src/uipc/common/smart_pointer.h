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
}  // namespace uipc

#include "details/smart_pointer.inl"