#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/span.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/unordered_map.h>
#include <uipc/geometry/geometry.h>

namespace uipc::backend
{
class SanityCheckMessageVisitor;
}

namespace uipc::core
{
class Scene;

enum class SanityCheckResult : int
{
    Success = 0,
    Warning = 1,
    Error   = 2
};

class UIPC_CORE_API SanityCheckMessage
{
  public:
    SanityCheckMessage() = default;

    U64               id() const noexcept;
    std::string_view  name() const noexcept;
    SanityCheckResult result() const noexcept;
    std::string_view  message() const noexcept;
    const unordered_map<std::string, S<geometry::Geometry>>& geometries() const noexcept;
    bool is_empty() const noexcept;

  private:
    friend class backend::SanityCheckMessageVisitor;
    U64               m_id = 0;
    std::string       m_name;
    SanityCheckResult m_result = SanityCheckResult::Success;
    std::string       m_message;
    unordered_map<std::string, S<geometry::Geometry>> m_geometries;
};

class UIPC_CORE_API SanityCheckMessageCollection
{
  public:
    SanityCheckMessageCollection() = default;
    const auto& messages() const noexcept { return m_messages; }
    auto&       messages() noexcept { return m_messages; }

  private:
    unordered_map<U64, S<SanityCheckMessage>> m_messages;
};

class UIPC_CORE_API ISanityChecker
{
  public:
    virtual ~ISanityChecker() = default;
    virtual void      build();
    U64               id() const noexcept;
    SanityCheckResult check(SanityCheckMessage& msg);
    std::string       name() const noexcept;

  protected:
    virtual U64               get_id() const noexcept           = 0;
    virtual std::string       get_name() const noexcept         = 0;
    virtual SanityCheckResult do_check(SanityCheckMessage& msg) = 0;
};

class UIPC_CORE_API SanityCheckerCollectionCreateInfo
{
  public:
    std::string_view workspace;
};

class UIPC_CORE_API ISanityCheckerCollection
{
  public:
    virtual ~ISanityCheckerCollection() = default;
    virtual void build(Scene& s)        = 0;
    virtual SanityCheckResult check(SanityCheckMessageCollection& msg) const = 0;
};
}  // namespace uipc::core
