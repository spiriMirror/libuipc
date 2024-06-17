#include <uipc/geometry/topo_elements.h>
#include <uipc/common/range.h>

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

void ITopoElements::reorder(span<const SizeT> O)
{
    do_reorder(O);
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
Json ITopoElements::to_json(SizeT i) const
{
    return do_to_json(i);
}

Json ITopoElements::to_json() const
{
    Json j;
    for(auto i : range(size()))
        j.push_back(to_json(i));
    return j;
}
}  // namespace uipc::geometry
