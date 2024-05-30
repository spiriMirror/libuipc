#include <uipc/common/span.h>
#include <uipc/backend/buffer_view.h>
namespace uipc::geometry
{
/**
 * @brief An interface to create a non-const view
 */
template <typename T, typename U>
span<T> view(U&)
{
    static_assert("Not implemented");
}
/**
 * @brief An interface to get the backend buffer view
 */
template <typename T, typename U>
backend::BufferView backend_view(U&)
{
    static_assert("Not implemented");
}
}  // namespace uipc::geometries
