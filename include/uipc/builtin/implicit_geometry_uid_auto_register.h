#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ImplicitGeometryUIDAutoRegister
{
  public:
    class Creator
    {
      public:
        Creator(std::function<list<UIDInfo>()> creator, std::string_view file, int line) noexcept
            : m_creator{std::move(creator)}
            , m_file{file}
            , m_line{line}
        {
        }

        list<UIDInfo> operator()() const { return m_creator(); }

        std::string_view file() const noexcept { return m_file; }
        int              line() const noexcept { return m_line; }

      private:
        std::function<list<UIDInfo>()> m_creator;
        std::string_view               m_file;
        int                            m_line = 0;
    };

    ImplicitGeometryUIDAutoRegister(Creator creator) noexcept;

  private:
    friend class ImplicitGeometryUIDCollection;
    static list<Creator>& creators() noexcept;
};
}  // namespace uipc::builtin

#define REGISTER_IMPLICIT_GEOMETRY_UIDS_INTERNAL(counter)                                                 \
    namespace auto_register                                                                               \
    {                                                                                                     \
        static ::uipc::list<::uipc::builtin::UIDInfo> ImplicitGeometryUIDAutoRegisterFunction##counter(); \
        static ::uipc::builtin::ImplicitGeometryUIDAutoRegister ImplicitGeometryUIDAutoRegister##counter{ \
            ::uipc::builtin::ImplicitGeometryUIDAutoRegister::Creator{                                    \
                ImplicitGeometryUIDAutoRegisterFunction##counter, __FILE__, __LINE__}};                   \
    }                                                                                                     \
    static ::uipc::list<::uipc::builtin::UIDInfo> auto_register::ImplicitGeometryUIDAutoRegisterFunction##counter()

/**
 * @brief Register ImplicitGeometryUIDs.
 * 
 * Example:
 * 
 * ```c++
 *  REGISTER_IMPLICIT_GEOMETRY_UIDS()
 *  {
 *      using namespace uipc::builtin;
 *      list<UIDInfo> uids;
 *      return uids;
 *  }
 * ```
 */
#define REGISTER_IMPLICIT_GEOMETRY_UIDS(...)                                   \
    REGISTER_IMPLICIT_GEOMETRY_UIDS_INTERNAL(__COUNTER__)