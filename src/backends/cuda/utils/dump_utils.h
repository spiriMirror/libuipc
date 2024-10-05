#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <uipc/common/vector.h>
#include <fmt/printf.h>
#include <uipc/common/log.h>
#include <fstream>

namespace uipc::backend::cuda
{
class BufferDump
{
    static constexpr std::size_t magic_number = 0xc2663291fdf3;
    static_assert(magic_number < ~0ull, "Magic number is too large");

    std::vector<std::byte> byte_buffer;

  public:
    BufferDump() = default;

    void clean_up()
    {
        byte_buffer.clear();
        byte_buffer.shrink_to_fit();
    }

    template <typename T>
    span<T> view()
    {
        return span<T>{reinterpret_cast<T*>(byte_buffer.data()),
                       byte_buffer.size() / sizeof(T)};
    }

    /**
     * @brief Dump host vector-like buffer to file
     * 
     * @return true for success, false for failure
	 */
    template <typename T>
    bool dump(std::string_view path, span<const T> buffer)
    {
        std::size_t size_bytes = buffer.size() * sizeof(T);
        byte_buffer.resize(size_bytes);
        std::memcpy(byte_buffer.data(), buffer.data(), size_bytes);
        return dump_(path);
    };

    /**
     * @brief Dump device buffer to file
     * 
     * @return true for success, false for failure
     */
    template <typename T>
    bool dump(std::string_view path, muda::CBufferView<T> buffer)
    {
        std::size_t size_bytes = buffer.size() * sizeof(T);
        byte_buffer.resize(size_bytes);
        buffer.copy_to((T*)byte_buffer.data());
        return dump_(path);
    }

    /**
     * @brief Dump device buffer to file
     * 
     * @return true for success, false for failure
     */
    template <typename T>
    bool dump(std::string_view path, const muda::DeviceBuffer<T>& buffer)
    {
        std::size_t size_bytes = buffer.size() * sizeof(T);
        byte_buffer.resize(size_bytes);
        buffer.view().copy_to((T*)byte_buffer.data());
        return dump_(path);
    }

    /**
     * @brief Load buffer from file
     * 
     * To use this function, copy the data to `BufferDump::view()` first.
     * 
     * @return true for success, false for failure
     */
    bool load(std::string_view path) { return load_(path); }

    /**
     * @brief Apply the cached data to device buffer
     * 
     * @param buffer
     */
    template <typename T>
    void apply_to(muda::DeviceBuffer<T>& buffer)
    {
        buffer.resize(byte_buffer.size() / sizeof(T));
        buffer.view().copy_from((const T*)byte_buffer.data());
    }

    /**
     * @brief Apply the cached data to host vector
     * 
     * @param buffer
     */
    template <typename T>
    void apply_to(vector<T>& buffer)
    {
        buffer.resize(byte_buffer.size() / sizeof(T));
        std::memcpy(buffer.data(), byte_buffer.data(), byte_buffer.size());
    }

  private:
    bool dump_(std::string_view path)
    {
        auto size_bytes = byte_buffer.size();

        std::ofstream ofs(std::string{path}, std::ios::binary);
        if(!ofs.is_open())
        {
            spdlog::warn("Failed to open file {} when dumping buffer", path);
            return false;
        }

        // 0: Fill 8 bytes magic number for alignment
        ofs.write((const char*)&magic_number, sizeof(magic_number));

        // 8: Write size of buffer
        ofs.write((char*)&size_bytes, sizeof(size_bytes));

        // 16: Write buffer
        ofs.write((char*)byte_buffer.data(), size_bytes);

        ofs.close();
        return true;
    }

    bool load_(std::string_view path)
    {
        std::ifstream ifs(std::string{path}, std::ios::binary);
        if(!ifs.is_open())
        {
            spdlog::warn("Failed to open file {} when loading buffer", path);
            return false;
        }

        // 0: Read magic number
        std::size_t magic;
        ifs.read((char*)&magic, sizeof(magic));
        if(magic != magic_number)
        {
            spdlog::warn("Magic number mismatch when loading buffer, invalid dump.");
            return false;
        }

        // 8: Read size of buffer
        std::size_t size_bytes;
        ifs.read((char*)&size_bytes, sizeof(size_bytes));

        // 16: Read buffer
        byte_buffer.resize(size_bytes);
        ifs.read((char*)byte_buffer.data(), size_bytes);

        ifs.close();
        return true;
    }
};
}  // namespace uipc::backend::cuda
