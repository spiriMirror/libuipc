#include <uipc/builtin/constitution_uid_register.h>

namespace uipc::builtin
{
/*
struct ConstitutionUIDInfo
{
    U64         uid;
    std::string name;
    std::string author;
    std::string email;
    std::string website;
    std::string description;
};
*/

ConstitutionUIDRegister::ConstitutionUIDRegister()
{
    create(ConstitutionUIDInfo{.uid = 0, .name = "Empty"});

    // create 8 AffineBody constitution uids
    for(U64 i = 1; i <= 8; ++i)
        create(ConstitutionUIDInfo{.uid = i, .name = "AffineBody"});


}
}  // namespace uipc::builtin
