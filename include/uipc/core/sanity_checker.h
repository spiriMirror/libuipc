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
    virtual void      build();
    U64               id() const noexcept;
    SanityCheckResult check();
    std::string       name() const noexcept;

  protected:
    virtual U64               get_id() const noexcept   = 0;
    virtual std::string       get_name() const noexcept = 0;
    virtual SanityCheckResult do_check()                = 0;
};

class UIPC_CORE_API SanityCheckerCollectionCreateInfo
{
  public:
    std::string_view workspace;
};

class UIPC_CORE_API ISanityCheckerCollection
{
  public:
    virtual ~ISanityCheckerCollection()       = default;
    virtual void              build(Scene& s) = 0;
    virtual SanityCheckResult check() const   = 0;
};
}  // namespace uipc::core
