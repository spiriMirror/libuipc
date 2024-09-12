#include <uipc/common/log.h>
namespace uipc
{
template <typename T>
void PmrDeleter<T>::operator()(T* ptr) const
{
    auto      resource = std::pmr::get_default_resource();
    Allocator alloc{resource};
    std::allocator_traits<Allocator>::destroy(alloc, ptr);
    std::allocator_traits<Allocator>::deallocate(alloc, ptr, 1);
}

template <typename T, typename... Args>
U<T> make_unique(Args&&... args)
{
    std::pmr::polymorphic_allocator<T> alloc;
    return U<T>(alloc.template new_object<T, Args...>(std::forward<Args>(args)...),
                PmrDeleter<T>{});
}

template <typename DstT, typename SrcT>
U<DstT> static_pointer_cast(U<SrcT>&& src)
{
    return U<DstT>(src.release());
}

template <typename T, typename... Args>
S<T> make_shared(Args&&... args)
{
    auto resource = std::pmr::get_default_resource();
    std::pmr::polymorphic_allocator<T> alloc{resource};
    return std::shared_ptr<T>(alloc.template new_object<T, Args...>(std::forward<Args>(args)...),
                              PmrDeleter<T>{});
}
}  // namespace uipc