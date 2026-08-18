#include <muda/ext/field/field.h>
#include <muda/ext/field/field_entry.h>
#include <muda/ext/field/field_builder.h>
#include <muda/ext/field/sub_field/aosoa_sub_field.h>
#include <muda/ext/field/sub_field/soa_sub_field.h>
#include <muda/ext/field/sub_field/aos_sub_field.h>
#include <muda/ext/field/sub_field_interface.h>
#include <muda/type_traits/type_label.h>

namespace muda
{
MUDA_INLINE SubField::SubField(Field& field, std::string_view name)
    : m_field(field)
    , m_name(name)
{
}

MUDA_INLINE SubField::~SubField() {}

MUDA_INLINE std::byte* SubField::data_buffer() const
{
    return m_interface->m_data_buffer;
}
MUDA_INLINE size_t SubField::num_entries() const
{
    return m_interface->m_entries.size();
}

template <FieldEntryLayout Layout>
MUDA_INLINE FieldBuilder<Layout> SubField::builder(FieldEntryLayoutInfo layout,
                                                   const FieldBuildOptions& options)
{
    if constexpr(Layout == FieldEntryLayout::RuntimeLayout)
    {
        return this->builder(layout, options);
    }
    else
    {
        m_interface = std::make_unique<SubFieldImpl<Layout>>(m_field);
        m_interface->m_layout_info = layout;
        return FieldBuilder<Layout>{*this, layout, options};
    }
}

MUDA_INLINE FieldBuilder<FieldEntryLayout::RuntimeLayout> SubField::builder(
    FieldEntryLayoutInfo layout, const FieldBuildOptions& options)
{
    switch(layout.layout())
    {
        case FieldEntryLayout::AoSoA:
            m_interface = std::make_unique<SubFieldImpl<FieldEntryLayout::AoSoA>>(m_field);
            break;
        case FieldEntryLayout::SoA:
            m_interface = std::make_unique<SubFieldImpl<FieldEntryLayout::SoA>>(m_field);
            break;
        case FieldEntryLayout::AoS:
            m_interface = std::make_unique<SubFieldImpl<FieldEntryLayout::AoS>>(m_field);
            break;
        default:
            MUDA_ERROR_WITH_LOCATION("Invalid layout type");
            break;
    }

    m_interface->m_layout_info = layout;
    return FieldBuilder<FieldEntryLayout::RuntimeLayout>{*this, layout, options};
}
template <typename T, FieldEntryLayout Layout, int M, int N>
auto SubField::create_entry(std::string_view     name,
                            FieldEntryLayoutInfo layout,
                            FieldEntryType       type,
                            uint2 shape) -> FieldEntry<T, Layout, M, N>&
{
    static_assert(muda::is_trivial_v<T>, R"(T must be trivial type, such as int/float/...
Or you need to label `YourType` as:
template <>
struct force_trivial<YourType>
{
    constexpr static bool value = true;
};)");
    auto ptr = new FieldEntry<T, Layout, M, N>(*this, layout, type, shape, name);
    m_interface->m_name_to_index[std::string{name}] = m_interface->m_entries.size();
    m_interface->m_entries.emplace_back(ptr);
    return *ptr;
}

MUDA_INLINE auto SubField::find_entry(std::string_view name) const -> FieldEntryBase*
{
    auto it = m_interface->m_name_to_index.find(std::string{name});
    if(it == m_interface->m_name_to_index.end())
        return nullptr;
    return (m_interface->m_entries[it->second]).get();
}

template <typename FieldEntryT>
MUDA_INLINE FieldEntryT* SubField::find_entry(std::string_view name) const
{
    static_assert(std::is_base_of_v<FieldEntryBase, FieldEntryT>,
                  "FieldEntryT must be derived from FieldEntryBase");

    auto ptr = find_entry(name);
    if(!ptr)
        return nullptr;
    return dynamic_cast<FieldEntryT*>(ptr);
}

MUDA_INLINE FieldBuilder<FieldEntryLayout::AoSoA> SubField::AoSoA(uint32_t innermost_array_size,
                                                                  const FieldBuildOptions& options)
{
    return builder<FieldEntryLayout::AoSoA>(
        FieldEntryLayoutInfo{FieldEntryLayout::AoSoA, innermost_array_size}, options);
}

MUDA_INLINE FieldBuilder<FieldEntryLayout::SoA> SubField::SoA(const FieldBuildOptions& options)
{
    return builder<FieldEntryLayout::SoA>(FieldEntryLayoutInfo{FieldEntryLayout::SoA, 0}, options);
}

MUDA_INLINE FieldBuilder<FieldEntryLayout::AoS> SubField::AoS(const FieldBuildOptions& options)
{
    return builder<FieldEntryLayout::AoS>(FieldEntryLayoutInfo{FieldEntryLayout::AoS, 0}, options);
}

MUDA_INLINE void SubField::build(const FieldBuildOptions& options)
{
    m_interface->m_build_options = options;
    MUDA_ASSERT(!m_is_built, "Field is already built!");
    m_interface->build_impl();
    m_is_built = true;
}

MUDA_INLINE bool SubField::allow_inplace_shrink() const
{
    return m_interface->allow_inplace_shrink();
}

MUDA_INLINE size_t SubField::size() const
{
    return m_interface ? m_interface->m_num_elements : 0;
}

MUDA_INLINE void SubField::resize(size_t num_elements)
{
    MUDA_ASSERT(m_is_built, "Field is not built yet!")
    m_interface->resize(num_elements);
    m_interface->m_num_elements = num_elements;
}
}  // namespace muda