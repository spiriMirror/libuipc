#include <uipc/geometry/topo_elements.h>

namespace uipc::geometry
{
SizeT ITopoElements::tuple_size() const noexcept
{
    return get_tuple_size();
}

SizeT ITopoElements::tuple_size(IndexT i) const noexcept
{
    return get_tuple_size(i);
}

SizeT ITopoElements::size() const noexcept
{
    return get_size();
}

void ITopoElements::resize(SizeT N)
{
    do_resize(N);
}

void ITopoElements::clear()
{
    do_clear();
}

void ITopoElements::reserve(SizeT N)
{
    do_reserve(N);
}

S<ITopoElements> ITopoElements::clone() const
{
    return do_clone();
}

backend::BufferView ITopoElements::backend_view() const noexcept
{
    return get_backend_view();
}

backend::BufferView backend_view(const ITopoElements& e) noexcept
{
    return e.backend_view();
}
}  // namespace uipc::geometries
