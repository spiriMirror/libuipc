#pragma once
#include <memory>
#include <muda/ext/field/field_build_options.h>
#include <muda/ext/field/field_entry_type.h>
#include <vector>
#include <unordered_map>
#include <muda/mstl/span.h>

namespace muda
{
class Field;
class FieldEntryBase;

class SubFieldInterface
{
    template <typename T>
    using U = std::unique_ptr<T>;
    friend class SubField;
    template <FieldEntryLayout Layout>
    friend class FieldBuilder;

  protected:
    Field&                                  m_field;
    std::vector<U<FieldEntryBase>>          m_entries;
    FieldEntryLayoutInfo                    m_layout_info;
    FieldBuildOptions                       m_build_options;
    std::unordered_map<std::string, size_t> m_name_to_index;
    size_t                                  m_num_elements     = 0;
    uint32_t                                m_struct_stride    = ~0;
    std::byte*                              m_data_buffer      = nullptr;
    size_t                                  m_data_buffer_size = 0;

    /***************************************************************************************************
                                            Subclass Implement
    ****************************************************************************************************/
    virtual void   build_impl()                                         = 0;
    virtual size_t require_total_buffer_byte_size(size_t element_count) = 0;
    virtual void   calculate_new_cores(std::byte*           byte_buffer,
                                       size_t               total_bytes,
                                       size_t               element_count,
                                       span<FieldEntryCore> new_cores)  = 0;
    virtual bool   allow_inplace_shrink() const { return true; }


    /***************************************************************************************************
                                            Common Utilities
    ****************************************************************************************************/
    const FieldEntryLayoutInfo& layout_info() const { return m_layout_info; }
    const FieldBuildOptions& build_options() const { return m_build_options; }
    size_t                   num_elements() const { return m_num_elements; }
    static uint32_t round_up(uint32_t total, uint32_t N);
    static uint32_t align(uint32_t offset, uint32_t size, uint32_t min_alignment, uint32_t max_alignment);
  public:
    SubFieldInterface(Field& field) MUDA_NOEXCEPT : m_field(field) {}
    virtual ~SubFieldInterface();

    // delete copy and move
    SubFieldInterface(const SubFieldInterface&)            = delete;
    SubFieldInterface& operator=(const SubFieldInterface&) = delete;
    SubFieldInterface(SubFieldInterface&&)                 = delete;
    SubFieldInterface& operator=(SubFieldInterface&&)      = delete;

  private:
    // used to resize the entry buffer.
    std::vector<FieldEntryCore>                   m_new_cores;
    std::vector<HostDeviceConfig<FieldEntryCore>> m_host_device_new_cores;

    /***************************************************************************************************
                                            Internal Utilities
    ****************************************************************************************************/
    void aync_upload_cores();
    void async_upload_temp_cores();
    template <typename F>  // F: void(std::byte* old_ptr, size_t old_size, std::byte* new_ptr, size_t new_size)
    void resize_data_buffer(size_t size, F&& func);

    /***************************************************************************************************
                                             SubField Using Only
    ****************************************************************************************************/
    void resize(size_t num_elements);
    void build();
};
}  // namespace muda

#include "details/sub_field_interface.inl"