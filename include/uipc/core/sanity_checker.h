#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/span.h>

namespace uipc::core
{
class Scene;

enum class SanityCheckResult : int
{
    Success = 0,
    Warning = 1,
    Error   = 2
};

class UIPC_CORE_API ISanityChecker
{
  public:
    virtual ~ISanityChecker() = default;
    virtual void      init();
    virtual void      deinit();
    U64               id() const noexcept;
    SanityCheckResult check();

  protected:
    virtual U64               get_id() const noexcept = 0;
    virtual SanityCheckResult do_check()              = 0;
};

class UIPC_CORE_API SanityCheckerCollectionCreateInfo
{
  public:
    std::string_view workspace;
};

class UIPC_CORE_API ISanityCheckerCollection
{
  public:
    virtual ~ISanityCheckerCollection()      = default;
    virtual void              init(Scene& s) = 0;
    virtual SanityCheckResult check() const  = 0;
};
}  // namespace uipc::core
