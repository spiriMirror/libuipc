#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/span.h>
namespace uipc::geometry
{
class UIPC_GEOMETRY_API SimplexUtils
{
  public:
    static bool is_same_edge(const Vector2i&, const Vector2i&) noexcept;
    static bool is_same_oriented_edge(const Vector2i&, const Vector2i&) noexcept;
    static bool is_same_tri(const Vector3i&, const Vector3i&) noexcept;
    static bool is_same_oriented_tri(const Vector3i&, const Vector3i&) noexcept;
    static bool is_same_tet(const Vector4i&, const Vector4i&) noexcept;
    static bool is_same_oriented_tet(const Vector4i&, const Vector4i&) noexcept;
    static bool compare_edge(const Vector2i&, const Vector2i&) noexcept;
    static bool compare_tri(const Vector3i&, const Vector3i&) noexcept;
    static bool compare_tet(const Vector4i&, const Vector4i&) noexcept;

    static void outward_tri_from_tet(span<const Vector3, 4> Vs, span<Vector3i, 4> Fs) noexcept;
};
}  // namespace uipc::geometry
