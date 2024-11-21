#pragma once
#include <uipc/common/type_define.h>

namespace uipc::geometry
{
// This class provides the quadrature rules for the triangle element.
//tex:
//$$
// \int_0^1 \int_0^{1-v} f(u,v) du dv
//$$
class TriangleQuatrature
{
  public:
    // f = 1
    static constexpr Float I_1_dudv = 1.0 / 2.0;

    // f = u
    static constexpr Float I_u_dudv = 1.0 / 6.0;
    // f = v
    static constexpr Float I_v_dudv = I_u_dudv;

    // f = u^2
    static constexpr Float I_uu_dudv = 1.0 / 12.0;
    // f = uv
    static constexpr Float I_uv_dudv = 1.0 / 24.0;
    // f = v^2
    static constexpr Float I_vv_dudv = I_uu_dudv;


    // f = u^3
    static constexpr Float I_uuu_dudv = 1.0 / 20.0;
    // f = u^2v
    static constexpr Float I_uuv_dudv = 1.0 / 60.0;
    // f = uv^2
    static constexpr Float I_uvv_dudv = I_uuv_dudv;
    // f = v^3
    static constexpr Float I_vvv_dudv = I_uuu_dudv;
};
}  // namespace uipc::geometry
