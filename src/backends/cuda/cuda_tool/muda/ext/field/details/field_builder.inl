#include <vector_functions.hpp>
#include <muda/ext/field/sub_field.h>

namespace muda
{
template <FieldEntryLayout Layout>
FieldBuilder<Layout>::~FieldBuilder()
{
    if(!m_is_built)
    {
        build();
    }
}

template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 1, 1>& FieldBuilder<Layout>::EntryProxy::scalar() &&
{
    return m_builder.create_entry<T, 1, 1>(m_name, FieldEntryType::Scalar);
}

template <FieldEntryLayout Layout>
template <typename T, int N>
FieldEntry<T, Layout, N, 1>& FieldBuilder<Layout>::EntryProxy::vector() &&
{
    static_assert(N >= 2, "When N == 1, use scalar() instead");
    return m_builder.create_entry<T, N, 1>(m_name, FieldEntryType::Vector);
}

template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 2, 1>& FieldBuilder<Layout>::EntryProxy::vector2() &&
{
    return std::move(*this).template vector<T, 2>();
}

template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 3, 1>& FieldBuilder<Layout>::EntryProxy::vector3() &&
{
    return std::move(*this).template vector<T, 3>();
}
template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 4, 1>& FieldBuilder<Layout>::EntryProxy::vector4() &&
{
    return std::move(*this).template vector<T, 4>();
}

template <FieldEntryLayout Layout>
template <typename T, int M, int N>
FieldEntry<T, Layout, M, N>& FieldBuilder<Layout>::EntryProxy::matrix() &&
{
    return m_builder.create_entry<T, M, N>(m_name, FieldEntryType::Matrix);
}

template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 2, 2>& FieldBuilder<Layout>::EntryProxy::matrix2x2() &&
{
    return std::move(*this).template matrix<T, 2, 2>();
}

template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 3, 3>& FieldBuilder<Layout>::EntryProxy::matrix3x3() &&
{
    return std::move(*this).template matrix<T, 3, 3>();
}

template <FieldEntryLayout Layout>
template <typename T>
FieldEntry<T, Layout, 4, 4>& FieldBuilder<Layout>::EntryProxy::matrix4x4() &&
{
    return std::move(*this).template matrix<T, 4, 4>();
}

template <FieldEntryLayout Layout>
template <typename T, int M, int N>
FieldEntry<T, Layout, M, N>& FieldBuilder<Layout>::create_entry(std::string_view name,
                                                                FieldEntryType type)
{
    return m_subfield.template create_entry<T, Layout, M, N>(
        name, m_layout, type, make_uint2(static_cast<uint32_t>(M), static_cast<uint32_t>(N)));
}

template <FieldEntryLayout Layout>
auto FieldBuilder<Layout>::entry(std::string_view name) -> EntryProxy
{
    MUDA_ASSERT(m_single_entry == false, "Named entry and Anonymous entry should not appear together!")
    return EntryProxy{*this, name};
}

template <FieldEntryLayout Layout>
auto FieldBuilder<Layout>::entry() -> EntryProxy
{
    MUDA_ASSERT(m_subfield.num_entries() == 0,
                "Anonymous entry should be the only entry in a SubField!");
    m_single_entry = true;
    return EntryProxy{*this, m_subfield.name()};
}

template <FieldEntryLayout Layout>
void FieldBuilder<Layout>::build()
{
    m_subfield.build(m_options);
    m_is_built = true;
}
}  // namespace muda