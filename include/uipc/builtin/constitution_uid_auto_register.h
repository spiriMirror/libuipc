#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ConstitutionUIDAutoRegister
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

    ConstitutionUIDAutoRegister(Creator creator) noexcept;

  private:
    friend class ConstitutionUIDCollection;
    static list<Creator>& creators() noexcept;
};
}  // namespace uipc::builtin

#define REGISTER_CONSTITUTION_UIDS_INTERNAL(counter)                                                  \
    namespace auto_register                                                                           \
    {                                                                                                 \
        static ::uipc::list<::uipc::builtin::UIDInfo> ConstitutionUIDAutoRegisterFunction##counter(); \
        static ::uipc::builtin::ConstitutionUIDAutoRegister ConstitutionUIDAutoRegister##counter{     \
            ::uipc::builtin::ConstitutionUIDAutoRegister::Creator{                                    \
                ConstitutionUIDAutoRegisterFunction##counter, __FILE__, __LINE__}};                   \
    }                                                                                                 \
    static ::uipc::list<::uipc::builtin::UIDInfo> auto_register::ConstitutionUIDAutoRegisterFunction##counter()

/**
 * @brief Register ConstitutionUIDs.
 * 
 * Example:
 * 
 * ```c++
 *  REGISTER_CONSTITUTION_UIDS()
 *  {
 *      using namespace uipc::builtin;
 *      list<UIDInfo> uids;
 *      return uids;
 *  }
 * ```
 */
#define REGISTER_CONSTITUTION_UIDS(...)                                        \
    REGISTER_CONSTITUTION_UIDS_INTERNAL(__COUNTER__)
