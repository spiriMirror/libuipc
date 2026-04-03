#pragma once
#include <uipc/common/exception.h>

namespace uipc::cuda_sanity_check
{
class CudaSanityCheckerException : public uipc::Exception
{
  public:
    using uipc::Exception::Exception;
};
}  // namespace uipc::cuda_sanity_check
