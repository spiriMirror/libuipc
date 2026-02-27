#pragma once
#include <cuda_runtime.h>
#include <muda/literal/unit.h>
#include <muda/muda_def.h>
#include <muda/check/check_cuda_errors.h>
#include <unordered_map>
#include <string>
#include <muda/tools/string_pointer.h>
#include <vector>
#include <cstring>

namespace muda::details
{
class HostDeviceStringCache
{
    class StringLocation
    {
      public:
        size_t buffer_index = ~0;
        size_t offset       = ~0;
        size_t size         = ~0;
    };

    std::unordered_map<std::string, StringLocation> m_string_map;

    std::vector<char*> m_device_string_buffers;
    std::vector<char*> m_host_string_buffers;

    size_t m_current_buffer_offset;
    size_t m_buffer_size;

    StringPointer m_empty_string_pointer{};

  public:
    HostDeviceStringCache(size_t buffer_size = 4_M)
        : m_buffer_size(buffer_size)
        , m_current_buffer_offset(0)
    {
        m_device_string_buffers.reserve(32);
        m_host_string_buffers.reserve(32);

        char* s;
        checkCudaErrors(cudaMalloc(&s, m_buffer_size * sizeof(char)));
        m_device_string_buffers.emplace_back(s);
        m_host_string_buffers.emplace_back(new char[m_buffer_size]);

        m_empty_string_pointer = get_string_pointer("");  // insert empty string
    }
    ~HostDeviceStringCache()
    {
        for(auto s : m_device_string_buffers)
            cudaFree(s);
        for(auto s : m_host_string_buffers)
            delete[] s;
    }
    // delete copy
    HostDeviceStringCache(const HostDeviceStringCache&)            = delete;
    HostDeviceStringCache& operator=(const HostDeviceStringCache&) = delete;
    // move
    HostDeviceStringCache(HostDeviceStringCache&&)            = default;
    HostDeviceStringCache& operator=(HostDeviceStringCache&&) = default;

    StringPointer operator[](std::string_view s)
    {
        if(s.empty() || s == "")
        {
            return m_empty_string_pointer;
        }
        return get_string_pointer(s);
    }

  private:
    StringPointer get_string_pointer(std::string_view s)
    {
        auto         str           = std::string{s};
        auto         it            = m_string_map.find(str);
        char*        device_string = nullptr;
        char*        host_string   = nullptr;
        unsigned int str_length    = 0;

        if(it != m_string_map.end())  // cached
        {
            auto& loc = it->second;
            device_string = m_device_string_buffers[loc.buffer_index] + loc.offset;
            host_string = m_host_string_buffers[loc.buffer_index] + loc.offset;
            str_length  = static_cast<unsigned int>(loc.size - 1);
        }
        else  // need insert
        {
            auto  zero_end_length = str.size() + 1;
            auto& loc             = m_string_map[str];  // insert

            if(m_current_buffer_offset + zero_end_length > m_buffer_size)  // need new buffer
            {
                char* s;
                checkCudaErrors(cudaMalloc(&s, m_buffer_size * sizeof(char)));
                m_device_string_buffers.emplace_back(s);
                m_host_string_buffers.emplace_back(new char[m_buffer_size]);
                m_current_buffer_offset = 0;
            }

            auto device_buffer = m_device_string_buffers.back();
            auto host_buffer   = m_host_string_buffers.back();

            // copy string to host buffer (with '\0' end)
            host_buffer[m_current_buffer_offset + str.size()] = '\0';
            std::memcpy(host_buffer + m_current_buffer_offset, str.data(), str.size());

            // copy string from host buffer to device buffer
            checkCudaErrors(cudaMemcpy(device_buffer + m_current_buffer_offset,
                                       host_buffer + m_current_buffer_offset,
                                       str.size() + 1,
                                       cudaMemcpyHostToDevice));

            loc.buffer_index = m_host_string_buffers.size() - 1;
            loc.offset       = m_current_buffer_offset;
            loc.size         = zero_end_length;  // include '\0'

            m_current_buffer_offset += zero_end_length;

            device_string = device_buffer + loc.offset;
            host_string   = host_buffer + loc.offset;
            str_length    = static_cast<unsigned int>(loc.size - 1);
        }
        return StringPointer{device_string, host_string, str_length};
    }
};
}  // namespace muda::details