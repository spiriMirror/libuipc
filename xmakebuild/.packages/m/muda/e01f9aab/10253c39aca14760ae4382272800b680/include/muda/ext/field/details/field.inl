#include <muda/ext/field/sub_field.h>

namespace muda
{
MUDA_INLINE Field::Field()
    : m_string_cache{4_K}
{
}

MUDA_INLINE SubField& muda::Field::operator[](std::string_view name)
{
    auto iter = m_name_to_index.find(std::string{name});
    if(iter == m_name_to_index.end())
    {
        auto  id = m_sub_fields.size();
        auto& sub_field =
            m_sub_fields.emplace_back(std::make_unique<SubField>(*this, name));
        m_name_to_index.emplace(name, id);
        return *sub_field;
    }
    else
    {
        return *m_sub_fields[iter->second];
    }
}

MUDA_INLINE Field::~Field() {}
}  // namespace muda