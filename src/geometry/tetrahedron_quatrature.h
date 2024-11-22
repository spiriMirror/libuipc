//#pragma once
//#include <uipc/common/type_define.h>
//
//namespace uipc::geometry
//{
//// This class provides the quadrature rules for the triangle element.
////tex:
////$$
//// \int_0^1 \int_0^{1-w} \int_0^{1-v-w} f(u,v,w) du dv dw
////$$
//class TetrahedronQuatrature
//{
//  public:
//    // f = 1
//    static constexpr Float I_1_dudvdw = 1.0 / 6.0;
//
//    // f = u
//    static constexpr Float I_u_dudvdw = 1.0 / 24.0;
//    // f = v
//    static constexpr Float I_v_dudvdw = I_u_dudvdw;
//    // f = w
//    static constexpr Float I_w_dudvdw = I_u_dudvdw;
//
//    // f = u^2
//    static constexpr Float I_uu_dudvdw = 1.0 / 60.0;
//    // f = v^2
//    static constexpr Float I_vv_dudvdw = I_uu_dudvdw;
//    // f = w^2
//    static constexpr Float I_ww_dudvdw = I_uu_dudvdw;
//
//    // f = uv
//    static constexpr Float I_uv_dudvdw = 1.0 / 120.0;
//    // f = uw
//    static constexpr Float I_uw_dudvdw = I_uv_dudvdw;
//    // f = vw
//    static constexpr Float I_vw_dudvdw = I_uv_dudvdw;
//};
//}  // namespace uipc::geometry
