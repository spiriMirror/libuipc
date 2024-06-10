#include <uipc/geometry/attribute.h>
#include <iostream>
namespace uipc::geometry
{
SizeT IAttribute::size() const
{
    return get_size();
}
void IAttribute::resize(SizeT N)
{
    do_resize(N);
}
void IAttribute::reserve(SizeT N)
{
    do_reserve(N);
}

S<IAttribute> IAttribute::clone() const
{
    return do_clone();
}

S<IAttribute> IAttribute::clone_empty() const
{
    return do_clone_empty();
}

void IAttribute::clear()
{
    do_clear();
}

void IAttribute::reorder(span<const SizeT> O) noexcept
{
    do_reorder(O);
}

void IAttribute::copy_from(span<const SizeT> O, const IAttribute& other) noexcept
{
    do_copy_from(O, other);
}

backend::BufferView IAttribute::backend_view() const noexcept
{
    return get_backend_view();
}

backend::BufferView backend_view(const IAttribute& a) noexcept
{
    return a.backend_view();
}
}  // namespace uipc::geometry
