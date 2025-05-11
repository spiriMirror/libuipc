#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
std::string_view IAttributeSlot::name() const noexcept
{
    return get_name();
}

std::string_view IAttributeSlot::type_name() const noexcept
{
    return attribute().type_name();
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

Json IAttributeSlot::to_json(SizeT i) const
{
    return do_to_json(i);
}

Json IAttributeSlot::to_json() const
{
    Json j;
    j["name"]   = name();
    j["values"] = attribute().to_json();
    return j;
}

//const BufferInfo& IAttributeSlot::buffer_info() const
//{
//    return buffer_info();
//}

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

S<IAttributeSlot> IAttributeSlot::clone(std::string_view name, bool allow_destroy) const
{
    return do_clone(name, allow_destroy);
}

S<IAttributeSlot> IAttributeSlot::clone_empty(std::string_view name, bool allow_destroy) const
{
    return do_clone_empty(name, allow_destroy);
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