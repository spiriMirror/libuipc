#pragma once
#include <memory>
#include <string>
#include <muda/ext/field/field_entry_type.h>
#include <muda/ext/field/field_entry_layout.h>

namespace muda
{
class SubField;
template <typename T, FieldEntryLayout Layout, int M, int N>
class FieldEntry;

template <FieldEntryLayout Layout>
class FieldBuilder
{
  public:
    class EntryProxy
    {
        FieldBuilder<Layout>& m_builder;

        std::string m_name;

      public:
        EntryProxy(FieldBuilder<Layout>& builder, std::string_view name)
            : m_builder(builder)
            , m_name(name)
        {
        }
        template <typename T>
        FieldEntry<T, Layout, 1, 1>& scalar() &&;

        template <typename T, int N>
        FieldEntry<T, Layout, N, 1>& vector() &&;

        template <typename T>
        FieldEntry<T, Layout, 2, 1>& vector2() &&;
        template <typename T>
        FieldEntry<T, Layout, 3, 1>& vector3() &&;
        template <typename T>
        FieldEntry<T, Layout, 4, 1>& vector4() &&;

        template <typename T, int M, int N>
        FieldEntry<T, Layout, M, N>& matrix() &&;

        template <typename T>
        FieldEntry<T, Layout, 2, 2>& matrix2x2() &&;
        template <typename T>
        FieldEntry<T, Layout, 3, 3>& matrix3x3() &&;
        template <typename T>
        FieldEntry<T, Layout, 4, 4>& matrix4x4() &&;
    };

  private:
    friend class SubField;

    SubField&            m_subfield;
    FieldEntryLayoutInfo m_layout;
    FieldBuildOptions    m_options;
    bool                 m_single_entry = false;
    bool                 m_is_built     = false;

    FieldBuilder(SubField& subfield, FieldEntryLayoutInfo layout, const FieldBuildOptions& options)
        : m_subfield(subfield)
        , m_layout(layout)
        , m_options(options)
    {
        MUDA_ASSERT(Layout == FieldEntryLayout::RuntimeLayout || layout.layout() == Layout,
                    "Layout mismatching");
    }

    template <typename T, int M, int N>
    FieldEntry<T, Layout, M, N>& create_entry(std::string_view name, FieldEntryType type);

  public:
    EntryProxy entry(std::string_view name);
    EntryProxy entry();
    /// <summary>
    /// Finish building the field.
    /// </summary>
    /// <param name="options"></param>
    void build();

    ~FieldBuilder();
};
}  // namespace muda

#include "details/field_builder.inl"