#include <uipc/common/const_string_map.h>

namespace uipc
{
std::string_view ConstStringMap::operator[](std::string_view content)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    auto [it, inserted] = m_pool.emplace(content);
    return std::string_view(it->data(), it->size());
}
}  // namespace uipc