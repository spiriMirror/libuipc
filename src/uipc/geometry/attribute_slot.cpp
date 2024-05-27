#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
std::string_view IAttributeSlot::name() const
{
    return get_name();
}

bool IAttributeSlot::allow_destroy() const
{
    return get_allow_destroy();
}

bool IAttributeSlot::is_shared() const
{
    return use_count() != 1;
}

SizeT IAttributeSlot::size() const
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

IAttribute& IAttributeSlot::attribute()
{
    return get_attribute();
}

const IAttribute& IAttributeSlot::attribute() const
{
    return get_attribute();
}
}  // namespace uipc::geometry