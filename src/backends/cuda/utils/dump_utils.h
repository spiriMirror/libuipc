#pragma once
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>
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
        if(size_bytes)
            std::memcpy(byte_buffer.data(), buffer.data(), size_bytes);
        return dump_(path);
    };

    /**
     * @brief Dump device buffer to file
     * 
     * @return true for success, false for failure
     */
    template <typename T>
    bool dump(std::string_view path, cuda_tool::CBufferView<T> buffer)
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
    bool dump(std::string_view path, const cuda_tool::DeviceBuffer<T>& buffer)
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
    template <typename T>
    bool load(std::string_view path)
    {
        return load_(path, sizeof(T));
    }

    /**
     * @brief Apply the cached data to device buffer
     * 
     * @param buffer
     */
    template <typename T>
    void apply_to(cuda_tool::DeviceBuffer<T>& buffer)
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
        if(!byte_buffer.empty())
            std::memcpy(buffer.data(), byte_buffer.data(), byte_buffer.size());
    }

  private:
    bool dump_(std::string_view path)
    {
        auto size_bytes = byte_buffer.size();

        std::ofstream ofs(std::string{path}, std::ios::binary);
        if(!ofs.is_open())
        {
            logger::warn("Failed to open file {} when dumping buffer", path);
            return false;
        }

        // 0: Fill 8 bytes magic number for alignment
        ofs.write((const char*)&magic_number, sizeof(magic_number));

        // 8: Write size of buffer
        ofs.write((char*)&size_bytes, sizeof(size_bytes));

        // 16: Write buffer
        ofs.write((char*)byte_buffer.data(), size_bytes);

        ofs.close();
        return !ofs.fail();
    }

    bool load_(std::string_view path, size_t element_size)
    {
        std::ifstream ifs(std::string{path}, std::ios::binary | std::ios::ate);
        if(!ifs)
        {
            logger::warn("Failed to open file {} when loading buffer", path);
            return false;
        }
        const std::streamoff file_size = ifs.tellg();
        ifs.seekg(0);
        std::size_t magic = 0, size_bytes = 0;
        constexpr std::streamoff header_size = sizeof(magic) + sizeof(size_bytes);
        if(!ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic))
           || !ifs.read(reinterpret_cast<char*>(&size_bytes), sizeof(size_bytes))
           || magic != magic_number || file_size < header_size
           || static_cast<std::uintmax_t>(file_size - header_size) != size_bytes
           || size_bytes % element_size != 0)
        {
            logger::warn("Invalid header or payload size in checkpoint buffer {}", path);
            return false;
        }
        byte_buffer.resize(size_bytes);
        return size_bytes == 0
               || static_cast<bool>(
                   ifs.read(reinterpret_cast<char*>(byte_buffer.data()), size_bytes));
    }
};
}  // namespace uipc::backend::cuda
