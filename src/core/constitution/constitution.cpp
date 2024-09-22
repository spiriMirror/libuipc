#include <uipc/constitution/constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
U64 IConstitution::uid() const noexcept
{
    return get_uid();
}

std::string_view IConstitution::name() const noexcept
{
    return uid_info().name;
}

std::string_view IConstitution::type() const noexcept
{
    return uid_info().type;
}

const builtin::UIDInfo& IConstitution::uid_info() const noexcept
{
    return builtin::ConstitutionUIDCollection::instance().find(get_uid());
}
}  // namespace uipc::constitution
