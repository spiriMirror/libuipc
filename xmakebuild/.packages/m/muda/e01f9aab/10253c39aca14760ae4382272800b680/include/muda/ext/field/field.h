#pragma once
#include <memory>
#include <muda/tools/host_device_string_cache.h>
#include <muda/ext/field/field_entry_layout.h>

namespace muda
{
class FieldEntryBase;
template <typename T, FieldEntryLayout Layout, int M, int N>
class FieldEntry;
class SubField;
class SubFieldInterface;
template <FieldEntryLayout Layout>
class SubFieldImpl;

class Field
{
    template <typename T>
    using U = std::unique_ptr<T>;

    friend class SubField;
    friend class SubFieldInterface;
    template <FieldEntryLayout Layout>
    friend class SubFieldImpl;
    friend class FieldEntryBase;

    details::HostDeviceStringCache          m_string_cache;
    std::vector<U<SubField>>                m_sub_fields;
    std::unordered_map<std::string, size_t> m_name_to_index;

  public:
    using Layout = FieldEntryLayout;
    Field();
    ~Field();

    // sub field count
    size_t num_sub_fields() const { return m_sub_fields.size(); }

    // create or find a subfield
    SubField& operator[](std::string_view name);
};
}  // namespace muda

#include "details/field.inl"