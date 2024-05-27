#include <uipc/common/span.h>
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
}  // namespace uipc::geometry
