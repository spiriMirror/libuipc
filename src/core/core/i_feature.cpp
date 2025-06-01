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

std::string_view Feature::get_type_name() const
{
    if(m_type_name.empty())
    {
        m_type_name = uipc::demangle(typeid(*this).name());
    }
    return m_type_name;
}
}  // namespace uipc::core
