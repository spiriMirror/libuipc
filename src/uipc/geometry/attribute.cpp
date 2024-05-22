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

void IAttribute::clear()
{
    do_clear();
}
}  // namespace uipc::geometry
