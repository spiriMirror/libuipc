#include <uipc/geometry/utils/simplex_utils.h>
#include <uipc/common/zip.h>
#include <algorithm>
#include <Eigen/Geometry>
#include <uipc/common/enumerate.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
bool SimplexUtils::is_same_edge(const Vector2i& A, const Vector2i& B) noexcept
{
    if(A == B)  // quick check
        return true;
    auto SA = A, SB = B;
    std::ranges::sort(SA);
    std::ranges::sort(SB);
    return SA == SB;
}

bool SimplexUtils::is_same_oriented_edge(const Vector2i& A, const Vector2i& B) noexcept
{
    return A == B;
}

bool SimplexUtils::is_same_tri(const Vector3i& A, const Vector3i& B) noexcept
{
    if(A == B)  // quick check
        return true;
    auto SA = A, SB = B;
    std::ranges::sort(SA);
    std::ranges::sort(SB);
    return SA == SB;
}

bool SimplexUtils::is_same_oriented_tri(const Vector3i& A, const Vector3i& B) noexcept
{
    if(A == B)  // quick check
        return true;

    auto RA = A;
    std::ranges::rotate_copy(A, A.begin() + 1, RA.begin());
    if(RA == B)
        return true;
    std::ranges::rotate(RA, RA.begin() + 1);
    if(RA == B)
        return true;
    return false;
}

bool SimplexUtils::is_same_tet(const Vector4i& A, const Vector4i& B) noexcept
{
    if(A == B)  // quick check
        return true;
    auto SA = A, SB = B;
    std::ranges::sort(SA);
    std::ranges::sort(SB);
    return SA == SB;
}

bool SimplexUtils::is_same_oriented_tet(const Vector4i& A, const Vector4i& B) noexcept
{
    if(A == B)  // quick check
        return true;

    auto RA = A;
    std::ranges::rotate_copy(A, A.begin() + 1, RA.begin());
    if(RA == B)
        return true;
    std::ranges::rotate(RA, RA.begin() + 1);
    if(RA == B)
        return true;
    std::ranges::rotate(RA, RA.begin() + 1);
    if(RA == B)
        return true;
    return false;
}

bool SimplexUtils::compare_edge(const Vector2i& A, const Vector2i& B) noexcept
{
    auto SA = A, SB = B;
    std::ranges::sort(SA);
    std::ranges::sort(SB);
    return SA(0) < SB(0) || (SA(0) == SB(0) && SA(1) < SB(1));
}

bool SimplexUtils::compare_tri(const Vector3i& A, const Vector3i& B) noexcept
{
    auto SA = A, SB = B;
    std::ranges::sort(SA);
    std::ranges::sort(SB);
    return SA(0) < SB(0)
           || (SA(0) == SB(0) && (SA(1) < SB(1) || (SA(1) == SB(1) && SA(2) < SB(2))));
}

bool SimplexUtils::compare_tet(const Vector4i& A, const Vector4i& B) noexcept
{
    auto SA = A, SB = B;
    std::ranges::sort(SA);
    std::ranges::sort(SB);
    return SA(0) < SB(0) || (SA(0) == SB(0) && SA(1) < SB(1))
           || (SA(0) == SB(0) && SA(1) == SB(1) && SA(2) < SB(2));
}

void SimplexUtils::outward_tri_from_tet(span<const Vector3, 4> Vs, span<Vector3i, 4> Fs) noexcept
{
    auto det = ((Vs[1] - Vs[0]).cross(Vs[2] - Vs[0])).dot(Vs[3] - Vs[0]);

    UIPC_ASSERT(det >= 0, "The tetrahedron is not positive oriented, now we don't support this case. TODO: ...");

    // initial values
    Fs[0] = Vector3i{0, 1, 2};
    Fs[1] = Vector3i{0, 1, 3};
    Fs[2] = Vector3i{0, 2, 3};
    Fs[3] = Vector3i{1, 2, 3};

    std::array<IndexT, 4> op_vert = {3, 2, 1, 0};

    auto Normal = [&](const Vector3i& I) -> Vector3
    {
        Vector3 V = Vs[I(0)];
        Vector3 U = Vs[I(1)];
        Vector3 W = Vs[I(2)];
        return (U - V).cross(W - V);
    };

    for(auto&& [i, t] : enumerate(Fs))
    {
        auto N = Normal(t);
        auto V = Vs[op_vert[i]];  // the opposite vertex
        auto P = Vs[t(0)];

        if((V - P).dot(N) > 0)
        {
            std::swap(t(1), t(2));  // swap the vertices to make the normal point outwards
        }
    }
}
}  // namespace uipc::geometry
