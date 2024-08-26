#include <contact_system/simplex_contact_constraint.h>

// Encoding Rules:
// ---------------------
// EE: ++++         (F)
// PE in EE: +++-   (E)
// PP in EE: ++--   (C)
// ---------------------
// PT: -+++         (7)
// PE in PT: -++-   (6)
// PP in PT: -+--   (4)
// ---------------------
// PE: +--[+]       (9)
// PP in PE: ++-[+] (D)
// ---------------------
// PP: --[++]       (3)
// ---------------------
// None: [----]     (0)
// ---------------------
// Rest:  (1,2,5,8,A,B)

namespace uipc::backend::cuda
{
namespace simplex_contact_constraint::detail
{
    UIPC_GENERIC inline IndexT make_neg(IndexT I) noexcept
    {
        MUDA_ASSERT(I >= 0, "invalid input I=%d, assume >= 0", I);
        return -I - 1;
    }

    UIPC_GENERIC inline IndexT make_pos(IndexT I) noexcept
    {
        MUDA_ASSERT(I < 0, "invalid input I=%d, assume < 0", I);
        return -I - 1;
    }

    UIPC_GENERIC inline char num_of(const Vector4i& data) noexcept
    {
        char num = 0;
#pragma unroll
        for(int i = 0; i < 4; ++i)
        {
            if(data[i] >= 0)
                num |= 1 << i;
        }
        return num;
    }

    // Magic Positive Value to fill the last element of PE and PP
    constexpr IndexT MagicPositive = std::numeric_limits<IndexT>::max();
    // Magic Negative Value to fill None
    constexpr IndexT MagicNegative = std::numeric_limits<IndexT>::min();
}  // namespace simplex_contact_constraint::detail

// EE: ++++
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::EE(const Vector2i& e0,
                                                                   const Vector2i& e1) noexcept
{
    return SimplexContactConstraint({
        e0[0],  // +
        e0[1],  // +
        e1[0],  // +
        e1[1]   // +
    });
}

// PE in EE: +++-
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PE_in_EE(
    IndexT p, const Vector2i& e, IndexT inactive_p) noexcept
{
    using namespace simplex_contact_constraint::detail;

    return SimplexContactConstraint({
        p,                    // +
        e[0],                 // +
        e[1],                 // +
        make_neg(inactive_p)  // -
    });
}

// PP in EE: ++--
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PP_in_EE(
    IndexT p_e0, IndexT p_e1, IndexT inactive_p_e0, IndexT inactive_p_e1) noexcept
{
    using namespace simplex_contact_constraint::detail;

    return SimplexContactConstraint({
        p_e0,                     // +
        p_e1,                     // +
        make_neg(inactive_p_e0),  // -
        make_neg(inactive_p_e1)   // -
    });
}

// PT: -+++
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PT(IndexT p, const Vector3i& t) noexcept
{
    using namespace simplex_contact_constraint::detail;

    return SimplexContactConstraint({
        make_neg(p),  // -
        t[0],         // +
        t[1],         // +
        t[2]          // +
    });
}

// PE in PT: -++-
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PE_in_PT(
    IndexT p, const Vector2i& e, IndexT inactive_p) noexcept
{
    using namespace simplex_contact_constraint::detail;
    return SimplexContactConstraint({
        make_neg(p),          // -
        e[0],                 // +
        e[1],                 // +
        make_neg(inactive_p)  // -
    });
}

// PP in PT: -+--
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PP_in_PT(
    IndexT p, IndexT p_t, const Vector2i& inactive_e) noexcept
{
    using namespace simplex_contact_constraint::detail;
    return SimplexContactConstraint({
        make_neg(p),              // -
        p_t,                      // +
        make_neg(inactive_e[0]),  // -
        make_neg(inactive_e[1])   // -
    });
}

// PE: +--[+]
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PE(IndexT p, const Vector2i& e) noexcept
{
    using namespace simplex_contact_constraint::detail;
    return SimplexContactConstraint({
        p,             // +
        e[0],          // -
        e[1],          // -
        MagicPositive  // [+]
    });
}

// PP in PE: ++-[+]
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PP_in_PE(IndexT p,
                                                                         IndexT p_e,
                                                                         IndexT inactive_p) noexcept
{
    using namespace simplex_contact_constraint::detail;
    return SimplexContactConstraint({
        p,                     // +
        p_e,                   // +
        make_neg(inactive_p),  // -
        MagicPositive          // [+]
    });
}

// PP: --[++]
UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::PP(IndexT p0, IndexT p1) noexcept
{
    using namespace simplex_contact_constraint::detail;
    return SimplexContactConstraint({
        make_neg(p0),   // -
        make_neg(p1),   // -
        MagicPositive,  // [+]
        MagicPositive   // [+]
    });
}

UIPC_GENERIC SimplexContactConstraint SimplexContactConstraint::None() noexcept
{
    using namespace simplex_contact_constraint::detail;
    return SimplexContactConstraint({
        MagicNegative,  // [----]
        MagicNegative,  // [----]
        MagicNegative,  // [----]
        MagicNegative   // [----]
    });
}

UIPC_GENERIC auto SimplexContactConstraint::type() const noexcept -> Type
{
    using namespace simplex_contact_constraint::detail;

    const char num = num_of(m_data);
    switch(num)
    {
        case 0x0F:
            return Type::EE;
        case 0x0E:
            return Type::PE_in_EE;
        case 0x0C:
            return Type::PP_in_EE;
        case 0x07:
            return Type::PT;
        case 0x06:
            return Type::PE_in_PT;
        case 0x04:
            return Type::PP_in_PT;
        case 0x09:
            return Type::PE;
        case 0x0D:
            return Type::PP_in_PE;
        case 0x03:
            return Type::PP;
        case 0x00:
            return Type::None;
        default:
            MUDA_ERROR_WITH_LOCATION("invalid contraint type=%d", num);
            return Type::None;
    }
}


// EE: ++++
UIPC_GENERIC void SimplexContactConstraint::as_EE(Vector2i& e0, Vector2i& e1) const noexcept
{
    MUDA_ASSERT(type() == Type::EE, "assume Type::EE(%d), your type=%d", Type::EE, type());

    e0[0] = m_data[0];  // +
    e0[1] = m_data[1];  // +
    e1[0] = m_data[2];  // +
    e1[1] = m_data[3];  // +
}

// PE in EE: +++-
UIPC_GENERIC void SimplexContactConstraint::as_PE_in_EE(IndexT&   p,
                                                        Vector2i& e,
                                                        IndexT& inactive_p) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PE_in_EE,
                "assume Type::PE_in_EE(%d), your type=%d",
                Type::PE_in_EE,
                type());

    p          = m_data[0];            // +
    e[0]       = m_data[1];            // +
    e[1]       = m_data[2];            // +
    inactive_p = make_pos(m_data[3]);  // -
}

// PP in EE: ++--
UIPC_GENERIC void SimplexContactConstraint::as_PP_in_EE(IndexT& p_e0,
                                                        IndexT& p_e1,
                                                        IndexT& inactive_p_e0,
                                                        IndexT& inactive_p_e1) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PP_in_EE,
                "assume Type::PP_in_EE(%d), your type=%d",
                Type::PP_in_EE,
                type());

    p_e0          = m_data[0];            // +
    p_e1          = m_data[1];            // +
    inactive_p_e0 = make_pos(m_data[2]);  // -
    inactive_p_e1 = make_pos(m_data[3]);  // -
}

// PT: -+++
UIPC_GENERIC void SimplexContactConstraint::as_PT(IndexT& p, Vector3i& t) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PT, "assume Type::PT(%d), your type=%d", Type::PT, type());

    p    = make_pos(m_data[0]);  // -
    t[0] = m_data[1];            // +
    t[1] = m_data[2];            // +
    t[2] = m_data[3];            // +
}

// PE in PT: -++-
UIPC_GENERIC void SimplexContactConstraint::as_PE_in_PT(IndexT&   p,
                                                        Vector2i& e,
                                                        IndexT& inactive_p) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PE_in_PT,
                "assume Type::PE_in_PT(%d), your type=%d",
                Type::PE_in_PT,
                type());

    p          = make_pos(m_data[0]);  // -
    e[0]       = m_data[1];            // +
    e[1]       = m_data[2];            // +
    inactive_p = make_pos(m_data[3]);  // -
}

// PP in PT: -+--
UIPC_GENERIC void SimplexContactConstraint::as_PP_in_PT(IndexT& p,
                                                        IndexT& p_t,
                                                        Vector2i& inactive_e) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PP_in_PT,
                "assume Type::PP_in_PT(%d), your type=%d",
                Type::PP_in_PT,
                type());

    p          = make_pos(m_data[0]);                         // -
    p_t        = m_data[1];                                   // +
    inactive_e = {make_pos(m_data[2]), make_pos(m_data[3])};  // -
}

// PE: +--[+]
UIPC_GENERIC void SimplexContactConstraint::as_PE(IndexT& p, Vector2i& e) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PE, "assume Type::PE(%d), your type=%d", Type::PE, type());

    p    = m_data[0];  // +
    e[0] = m_data[1];  // -
    e[1] = m_data[2];  // -

    // check magic positive value
    MUDA_ASSERT(m_data[3] == MagicPositive,
                "invalid magic positive value %d (assume %d)",
                m_data[3],
                MagicPositive);
}

// PP in PE: ++-[+]
UIPC_GENERIC void SimplexContactConstraint::as_PP_in_PE(IndexT& p,
                                                        IndexT& p_e,
                                                        IndexT& inactive_p) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PP_in_PE,
                "assume Type::PP_in_PE(%d), your type=%d",
                Type::PP_in_PE,
                type());

    p          = m_data[0];            // +
    p_e        = m_data[1];            // +
    inactive_p = make_pos(m_data[2]);  // -

    // check magic positive value
    MUDA_ASSERT(m_data[3] == MagicPositive,
                "invalid magic positive value %d (assume %d)",
                m_data[3],
                MagicPositive);
}

// PP: --[++]
UIPC_GENERIC void SimplexContactConstraint::as_PP(IndexT& p0, IndexT& p1) const noexcept
{
    using namespace simplex_contact_constraint::detail;
    MUDA_ASSERT(type() == Type::PP, "assume Type::PP(%d), your type=%d", Type::PP, type());

    p0 = make_pos(m_data[0]);  // -
    p1 = make_pos(m_data[1]);  // -

    // check magic positive value
    MUDA_ASSERT(m_data[2] == MagicPositive && m_data[3] == MagicPositive,
                "invalid magic positive value %d,%d (assume %d,%d)",
                m_data[2],
                m_data[3],
                MagicPositive,
                MagicPositive);
}

UIPC_GENERIC SimplexContactConstraint::SimplexContactConstraint(const Vector4i& data) noexcept
    : m_data(data)
{
}
}  // namespace uipc::backend::cuda
