#include <affine_body/utils.h>

namespace uipc::backend::cuda
{
UIPC_GENERIC Matrix3x3 q_to_A(const Vector12& q)
{
    Matrix3x3 A = Matrix3x3::Zero();
    A.row(0)    = q.segment<3>(3);
    A.row(1)    = q.segment<3>(6);
    A.row(2)    = q.segment<3>(9);
    return A;
}

UIPC_GENERIC Vector9 A_to_q(const Matrix3x3& A)
{
    Vector9 q       = Vector9::Zero();
    q.segment<3>(0) = A.row(0);
    q.segment<3>(3) = A.row(1);
    q.segment<3>(6) = A.row(2);
    return q;
}

UIPC_GENERIC Vector9 F_to_A(const Vector9& F)
{
    Vector9 A;
    A(0) = F(0);
    A(1) = F(3);
    A(2) = F(6);
    A(3) = F(1);
    A(4) = F(4);
    A(5) = F(7);
    A(6) = F(2);
    A(7) = F(5);
    A(8) = F(8);
    return A;
}

UIPC_GENERIC Matrix9x9 HF_to_HA(const Matrix9x9& HF)
{
    Matrix9x9 HA;
    HA(0, 0) = HF(0, 0);
    HA(0, 1) = HF(0, 3);
    HA(0, 2) = HF(0, 6);
    HA(0, 3) = HF(0, 1);
    HA(0, 4) = HF(0, 4);
    HA(0, 5) = HF(0, 7);
    HA(0, 6) = HF(0, 2);
    HA(0, 7) = HF(0, 5);
    HA(0, 8) = HF(0, 8);
    HA(1, 0) = HF(3, 0);
    HA(1, 1) = HF(3, 3);
    HA(1, 2) = HF(3, 6);
    HA(1, 3) = HF(3, 1);
    HA(1, 4) = HF(3, 4);
    HA(1, 5) = HF(3, 7);
    HA(1, 6) = HF(3, 2);
    HA(1, 7) = HF(3, 5);
    HA(1, 8) = HF(3, 8);
    HA(2, 0) = HF(6, 0);
    HA(2, 1) = HF(6, 3);
    HA(2, 2) = HF(6, 6);
    HA(2, 3) = HF(6, 1);
    HA(2, 4) = HF(6, 4);
    HA(2, 5) = HF(6, 7);
    HA(2, 6) = HF(6, 2);
    HA(2, 7) = HF(6, 5);
    HA(2, 8) = HF(6, 8);
    HA(3, 0) = HF(1, 0);
    HA(3, 1) = HF(1, 3);
    HA(3, 2) = HF(1, 6);
    HA(3, 3) = HF(1, 1);
    HA(3, 4) = HF(1, 4);
    HA(3, 5) = HF(1, 7);
    HA(3, 6) = HF(1, 2);
    HA(3, 7) = HF(1, 5);
    HA(3, 8) = HF(1, 8);
    HA(4, 0) = HF(4, 0);
    HA(4, 1) = HF(4, 3);
    HA(4, 2) = HF(4, 6);
    HA(4, 3) = HF(4, 1);
    HA(4, 4) = HF(4, 4);
    HA(4, 5) = HF(4, 7);
    HA(4, 6) = HF(4, 2);
    HA(4, 7) = HF(4, 5);
    HA(4, 8) = HF(4, 8);
    HA(5, 0) = HF(7, 0);
    HA(5, 1) = HF(7, 3);
    HA(5, 2) = HF(7, 6);
    HA(5, 3) = HF(7, 1);
    HA(5, 4) = HF(7, 4);
    HA(5, 5) = HF(7, 7);
    HA(5, 6) = HF(7, 2);
    HA(5, 7) = HF(7, 5);
    HA(5, 8) = HF(7, 8);
    HA(6, 0) = HF(2, 0);
    HA(6, 1) = HF(2, 3);
    HA(6, 2) = HF(2, 6);
    HA(6, 3) = HF(2, 1);
    HA(6, 4) = HF(2, 4);
    HA(6, 5) = HF(2, 7);
    HA(6, 6) = HF(2, 2);
    HA(6, 7) = HF(2, 5);
    HA(6, 8) = HF(2, 8);
    HA(7, 0) = HF(5, 0);
    HA(7, 1) = HF(5, 3);
    HA(7, 2) = HF(5, 6);
    HA(7, 3) = HF(5, 1);
    HA(7, 4) = HF(5, 4);
    HA(7, 5) = HF(5, 7);
    HA(7, 6) = HF(5, 2);
    HA(7, 7) = HF(5, 5);
    HA(7, 8) = HF(5, 8);
    HA(8, 0) = HF(8, 0);
    HA(8, 1) = HF(8, 3);
    HA(8, 2) = HF(8, 6);
    HA(8, 3) = HF(8, 1);
    HA(8, 4) = HF(8, 4);
    HA(8, 5) = HF(8, 7);
    HA(8, 6) = HF(8, 2);
    HA(8, 7) = HF(8, 5);
    HA(8, 8) = HF(8, 8);
    return HA;
}
UIPC_GENERIC Matrix4x4 q_to_transform(const Vector12& q)
{
    Matrix4x4 trans;
    // translation
    trans.block<3, 1>(0, 3) = q.segment<3>(0);
    // rotation
    trans.block<1, 3>(0, 0) = q.segment<3>(3).transpose();
    trans.block<1, 3>(1, 0) = q.segment<3>(6).transpose();
    trans.block<1, 3>(2, 0) = q.segment<3>(9).transpose();

    // last row
    trans.row(3) = Vector4{0, 0, 0, 1};
    return trans;
}

UIPC_GENERIC Vector12 transform_to_q(const Matrix4x4& trans)
{
    Vector12 q;
    q.segment<3>(0) = trans.block<3, 1>(0, 3);
    q.segment<3>(3) = trans.block<1, 3>(0, 0).transpose();
    q.segment<3>(6) = trans.block<1, 3>(1, 0).transpose();
    q.segment<3>(9) = trans.block<1, 3>(2, 0).transpose();

    return q;
}
}  // namespace uipc::backend::cuda
