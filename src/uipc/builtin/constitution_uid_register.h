#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/unordered_map.h>
#include <string>

namespace uipc::builtin
{
struct ConstitutionUIDInfo
{
    U64         uid;
    std::string name;
    std::string author;
    std::string email;
    std::string website;
    std::string description;
};
namespace details
{
    class ConstitutionUIDRegister
    {
      public:
        static constexpr U64 OfficialBuiltinUIDStart = 0;
        static constexpr U64 UserDefinedUIDStart     = 1 << 32;

        const ConstitutionUIDInfo& find(U64 uid) const;
        bool                       exists(U64 uid) const;

        static constexpr bool is_official_builtin_uid(U64 uid) noexcept
        {
            return uid < UserDefinedUIDStart;
        }

        static constexpr bool is_user_defined_uid(U64 uid) noexcept
        {
            return uid >= UserDefinedUIDStart;
        }


      private:
        unordered_map<U64, ConstitutionUIDInfo> m_uid_to_info;

      protected:
        void create(const ConstitutionUIDInfo& info);
    };
}  // namespace details

class ConstitutionUIDRegister : public details::ConstitutionUIDRegister
{
  public:
    static const ConstitutionUIDRegister& instance() noexcept;

  private:
    ConstitutionUIDRegister();
};
}  // namespace uipc::builtin
