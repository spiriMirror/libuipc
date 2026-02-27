namespace muda
{
MUDA_INLINE SubFieldInterface::~SubFieldInterface()
{
    if(m_data_buffer)
        Memory().free(m_data_buffer).wait();
};

template <typename F>
void SubFieldInterface::resize_data_buffer(size_t size, F&& func)
{
    if(m_data_buffer == nullptr)
    {
        Memory().alloc(&m_data_buffer, size).set(m_data_buffer, size, 0).wait();
        func(nullptr, 0, m_data_buffer, size);
        m_data_buffer_size = size;
    }
    else if(size > m_data_buffer_size)
    {
        auto old_ptr  = m_data_buffer;
        auto old_size = m_data_buffer_size;

        std::byte* new_ptr = nullptr;
        Memory().alloc(&new_ptr, size);
        func(old_ptr, old_size, new_ptr, size);
        Memory().free(old_ptr);

        // m_data_buffer should be updated at last
        // because the old field entry view need to
        // copy the data to new place.
        m_data_buffer      = new_ptr;
        m_data_buffer_size = size;
    }
    else
    {
        func(m_data_buffer, m_data_buffer_size, m_data_buffer, size);
    }
    wait_stream(nullptr);
}

MUDA_INLINE void SubFieldInterface::resize(size_t num_elements)
{
    m_new_cores.resize(m_entries.size());

    // let subclass fill the new field entry cores
    auto buffer_byte_size = require_total_buffer_byte_size(num_elements);

    resize_data_buffer(
        buffer_byte_size,
        [&](std::byte* old_ptr, size_t old_size, std::byte* new_ptr, size_t new_size)
        {
            for(size_t i = 0; i < m_entries.size(); i++)
            {
                auto& e    = m_entries[i];
                auto& c    = m_new_cores[i];
                c          = e->m_core;  // copy the old core to the new core
                c.m_buffer = new_ptr;    // set new ptr to new core
                c.m_info.elem_count = num_elements;  // set new element count
            }

            calculate_new_cores(new_ptr, new_size, num_elements, m_new_cores);
            async_upload_temp_cores();

            for(size_t i = 0; i < m_entries.size(); i++)
            {
                auto& e             = m_entries[i];
                auto& c             = m_new_cores[i];
                auto& host_device_c = m_host_device_new_cores[i];

                if(old_ptr)
                    e->async_copy_to_new_place(host_device_c.view());  // copy to new place (with layout)

                e->m_core = c;  // update the core
            }

            aync_upload_cores();
        });  // sync here.

    m_num_elements = num_elements;
}

MUDA_INLINE void SubFieldInterface::build()
{
    build_impl();
    // no need to upload, because there is no data at all
    // we wait until `resize()` to call: aync_upload_cores();
    // aync_upload_cores();
    wait_stream(nullptr);
}

MUDA_INLINE uint32_t SubFieldInterface::round_up(uint32_t x, uint32_t n)
{
    MUDA_ASSERT((n & (n - 1)) == 0, "n is not power of 2");
    return (x + n - 1) & ~(n - 1);
}

MUDA_INLINE uint32_t SubFieldInterface::align(uint32_t offset,
                                              uint32_t size,
                                              uint32_t min_alignment,
                                              uint32_t max_alignment)
{
    auto alignment = std::clamp(size, min_alignment, max_alignment);
    return round_up(offset, alignment);
}

MUDA_INLINE void muda::SubFieldInterface::aync_upload_cores()
{
    for(auto&& e : m_entries)
    {
        *e->m_host_device_core.host_data() = e->m_core;
        BufferLaunch().copy(e->m_host_device_core.buffer_view(), &e->m_core);
    }
}
MUDA_INLINE void muda::SubFieldInterface::async_upload_temp_cores()
{
    m_host_device_new_cores.resize(m_new_cores.size());
    for(size_t i = 0; i < m_new_cores.size(); ++i)
    {
        auto& core                    = m_new_cores[i];
        auto& host_device_core        = m_host_device_new_cores[i];
        *host_device_core.host_data() = core;
        BufferLaunch().copy(host_device_core.buffer_view(), &core);
    }
}
}  // namespace muda