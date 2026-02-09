#pragma once
#include <uipc/common/type_define.h>

namespace uipc::geometry::affine_body
{
inline Matrix4x4 q_to_transform(const Vector12& q)
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

inline Vector12 transform_to_q(const Matrix4x4& trans)
{
    Vector12 q;
    q.segment<3>(0) = trans.block<3, 1>(0, 3);
    q.segment<3>(3) = trans.block<1, 3>(0, 0).transpose();
    q.segment<3>(6) = trans.block<1, 3>(1, 0).transpose();
    q.segment<3>(9) = trans.block<1, 3>(2, 0).transpose();

    return q;
}

inline Matrix4x4 q_v_to_transform_v(const Vector12& q)
{
    Matrix4x4 trans;
    // translation
    trans.block<3, 1>(0, 3) = q.segment<3>(0);
    // rotation
    trans.block<1, 3>(0, 0) = q.segment<3>(3).transpose();
    trans.block<1, 3>(1, 0) = q.segment<3>(6).transpose();
    trans.block<1, 3>(2, 0) = q.segment<3>(9).transpose();

    // last row fill zero
    trans.row(3) = Vector4::Zero();
    return trans;
}

inline Vector12 transform_v_to_q_v(const Matrix4x4& transform_v)
{
    // the same to transform_to_q
    return transform_to_q(transform_v);
}
}  // namespace uipc::geometry::affine_body