#pragma once
#include <uipc/world/geometry_collection.h>

namespace uipc::world
{
SizeT IGeometryCollection::size() const noexcept
{
    return get_size();
}
void IGeometryCollection::clear() noexcept
{
    do_clear();
}
void IGeometryCollection::resize(SizeT size) noexcept
{
    do_resize(size);
}
void IGeometryCollection::reserve(SizeT size) noexcept
{
    do_reserve(size);
}
}  // namespace uipc::world
