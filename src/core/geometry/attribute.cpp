#include <uipc/geometry/attribute.h>
#include <iostream>
#include <uipc/common/range.h>

namespace uipc::geometry
{
SizeT IAttribute::size() const noexcept
{
    return get_size();
}

Json IAttribute::to_json(SizeT i) const noexcept
{
    return do_to_json(i);
}

Json IAttribute::to_json() const noexcept
{
    return do_to_json();
}

void IAttribute::from_json(const Json& j) noexcept
{
    do_from_json(j);
}

std::string_view IAttribute::type_name() const noexcept
{
    return get_type_name();
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

void IAttribute::copy_from(const IAttribute& other, const AttributeCopy& copy) noexcept
{
    do_copy_from(other, copy);
}
}  // namespace uipc::geometry
