#include <uipc/builtin/constitution_uid_register.h>

namespace uipc::builtin
{
ConstitutionUIDRegister::ConstitutionUIDRegister()
{
    create(UIDInfo{.uid = 0, .name = "Empty"});

    // create 8 AffineBody constitution uids
    create(UIDInfo{.uid = 1, .name = "AffineBody::OrthoPotential"});
    create(UIDInfo{.uid = 2, .name = "AffineBody::ARAP"});
    create(UIDInfo{.uid = 3, .name = "AffineBody"});
    create(UIDInfo{.uid = 4, .name = "AffineBody"});
    create(UIDInfo{.uid = 5, .name = "AffineBody"});
    create(UIDInfo{.uid = 6, .name = "AffineBody"});
    create(UIDInfo{.uid = 7, .name = "AffineBody"});
    create(UIDInfo{.uid = 8, .name = "AffineBody"});

    // FiniteElement constitution uids
    create(UIDInfo{.uid = 9, .name = "FiniteElement::StableNeoHookean"});
}
}  // namespace uipc::builtin
