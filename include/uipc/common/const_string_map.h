#pragma once
#include <string>
#include <string_view>
#include <uipc/common/unordered_set.h>

namespace uipc
{
class ConstStringMap
{
  public:
    std::string_view operator[](std::string_view content);

  private:
    std::unordered_set<std::string> m_pool;
    std::mutex                      m_mutex;
};
}  // namespace uipc
