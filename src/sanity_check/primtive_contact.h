#pragma once
#include <uipc/common/type_define.h>

namespace uipc::sanity_check
{
template <int M, int N>
bool need_contact(const ContactTabular&    contact_table,
                  const Vector<IndexT, M>& CIdLs,
                  const Vector<IndexT, N>& CIdRs)
{
    for(auto& L : CIdLs)
    {
        for(auto& R : CIdRs)
        {
            const core::ContactModel& model = contact_table.at(L, R);

            if(!model.is_enabled())
                return false;
        }
    }

    return true;
}

inline bool need_contact(const ContactTabular& contact_table, IndexT CIdL, IndexT CIdR)
{
    const core::ContactModel& model = contact_table.at(CIdL, CIdR);
    return model.is_enabled();
}

template <int N>
inline bool need_contact(const ContactTabular&    contact_table,
                         IndexT                   CIdL,
                         const Vector<IndexT, N>& CIdRs)
{
    for(auto& R : CIdRs)
    {
        const core::ContactModel& model = contact_table.at(CIdL, R);

        if(!model.is_enabled())
            return false;
    }

    return true;
}
}  // namespace uipc::sanity_check