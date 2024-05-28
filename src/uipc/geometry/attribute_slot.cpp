#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
std::string_view IAttributeSlot::name() const noexcept
{
    return get_name();
}

bool IAttributeSlot::allow_destroy() const noexcept
{
    return get_allow_destroy();
}

bool IAttributeSlot::is_shared() const noexcept
{
    return use_count() != 1;
}

SizeT IAttributeSlot::size() const noexcept
{
    return attribute().size();
}

void IAttributeSlot::make_owned()
{
    if(!is_shared())
        return;
    do_make_owned();
}

SizeT IAttributeSlot::use_count() const
{
    return get_use_count();
}

S<IAttributeSlot> IAttributeSlot::clone() const
{
    return do_clone();
}

IAttribute& IAttributeSlot::attribute() noexcept
{
    return get_attribute();
}

const IAttribute& IAttributeSlot::attribute() const noexcept
{
    return get_attribute();
}

backend::BufferView backend_view(const IAttributeSlot& a) noexcept
{
    return backend_view(a.attribute());
}
}  // namespace uipc::geometry