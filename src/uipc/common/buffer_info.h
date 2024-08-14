#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/vector.h>
namespace uipc
{
class BufferInfo
{
  public:
    void*         data;
    size_t        itemsize;
    vector<int64_t> shape;
    vector<int64_t> strides;
};
}  // namespace uipc
