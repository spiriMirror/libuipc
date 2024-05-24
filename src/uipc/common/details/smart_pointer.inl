#include <uipc/common/log.h>
namespace uipc
{
template <typename T>
T* P<T>::operator->() const
{
    auto ptr = Base::lock().get();
    UIPC_ASSERT(ptr, "Weak pointer expired");
    return ptr;
}

template <typename T>
T& P<T>::operator*() const
{
    auto ptr = Base::lock().get();
    UIPC_ASSERT(ptr, "Weak pointer expired");
    return *ptr;
}

template <typename T>
P<T>::operator bool() const
{
    return Base::lock().get();
}
}  // namespace uipc