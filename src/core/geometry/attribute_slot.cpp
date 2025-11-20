#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>
#include <uipc/geometry/attribute_debug_info.h>

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

Json IAttributeSlot::to_json() const
{
    Json j;
    j["name"]      = name();
    j["attribute"] = attribute().to_json();
    return j;
}

Json IAttributeSlot::to_json(SizeT i) const
{
    return attribute().to_json(i);
}

void IAttributeSlot::from_json_array(const Json& j) noexcept
{
    rw_access();
    attribute().from_json_array(j);
}

bool IAttributeSlot::is_evolving() const noexcept
{
    return get_is_evolving();
}

void IAttributeSlot::is_evolving(bool v) noexcept
{
    set_is_evolving(v);
}

TimePoint IAttributeSlot::last_modified() const noexcept
{
    return get_last_modified();
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

S<IAttributeSlot> IAttributeSlot::clone(std::string_view name, bool allow_destroy) const
{
    return do_clone(name, allow_destroy);
}

void IAttributeSlot::share_from(const IAttributeSlot& other) noexcept
{
    if(std::addressof(other) == this)
        return;
    last_modified(std::chrono::high_resolution_clock::now());
    do_share_from(other);
}

IAttribute& IAttributeSlot::attribute() noexcept
{
    return get_attribute();
}

const IAttribute& IAttributeSlot::attribute() const noexcept
{
    return get_attribute();
}

void IAttributeSlot::rw_access()
{
    last_modified(std::chrono::high_resolution_clock::now());
    make_owned();
}

void check_view(const IAttributeSlot* slot)
{
    UIPC_ASSERT(slot,
                "You are trying to access a nullptr attribute slot, please check if the attribute name is correct.\n"
                "The last attribute name (thread local) you tried to find is: {}",
                AttributeDebugInfo::thread_local_last_not_found_name());
}

void IAttributeSlot::last_modified(const TimePoint& tp)
{
    set_last_modified(tp);
}
}  // namespace uipc::geometry