#include <uipc/core/feature.h>
#include <uipc/common/demangle.h>

namespace uipc::core
{
std::string_view IFeature::name() const
{
    return get_name();
}

std::string_view IFeature::type_name() const
{
    return get_type_name();
}

void IFeature::on_required()
{
    do_on_required();
}

std::string_view Feature::get_type_name() const
{
    if(m_type_name.empty())
    {
        m_type_name = uipc::demangle(typeid(*this).name());
    }
    return m_type_name;
}

void Feature::do_on_required() const {}
}  // namespace uipc::core
