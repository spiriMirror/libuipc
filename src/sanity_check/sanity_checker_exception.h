#include <uipc/common/exception.h>

namespace uipc::sanity_check
{
    
class SanityCheckerException : public uipc::Exception
{
  public:
    using uipc::Exception::Exception;
};
}