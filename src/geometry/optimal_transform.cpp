#include <uipc/geometry/utils/optimal_transform.h>
#include <Eigen/Dense>
#include <uipc/common/range.h>

namespace uipc::geometry
{
static Matrix4x4 optimal_transform(const Matrix4x4&    ST,
                                   span<const Vector3> S,
                                   const Matrix4x4&    DT,
                                   span<const Vector3> D)
{
    UIPC_ASSERT(S.size() == D.size(),
                "The number of source points({}) and destination points({}) should be the same.",
                S.size(),
                D.size());

    UIPC_ASSERT(S.size() >= 4, "The number of points should be at least 4.");

    bool S_is_identity = ST.isIdentity();
    bool D_is_identity = DT.isIdentity();

    // \sum {J^T * J} q = \sum J^T * x
    Float     SI           = 0;
    Vector3   Sx_bar       = Vector3::Zero();
    Matrix3x3 Sx_bar_x_bar = Matrix3x3::Zero();
    Vector12  SJx          = Vector12::Zero();

    for(auto&& i : range(S.size()))
    {
        Vector3 x_bar =
            S_is_identity ? S[i] : ST.block<3, 3>(0, 0) * S[i] + ST.block<3, 1>(0, 3);
        Vector3 x =
            D_is_identity ? D[i] : DT.block<3, 3>(0, 0) * D[i] + DT.block<3, 1>(0, 3);

        SI += 1;
        Sx_bar += x_bar;
        Sx_bar_x_bar += x_bar * x_bar.transpose();
        SJx.segment<3>(0) += x;
        SJx.segment<3>(3) += x_bar * x(0);
        SJx.segment<3>(6) += x_bar * x(1);
        SJx.segment<3>(9) += x_bar * x(2);
    }

    Matrix12x12 A       = Matrix12x12::Zero();
    A.block<3, 3>(0, 0) = SI * Matrix3x3::Identity();

    A.block<1, 3>(0, 3) = Sx_bar.transpose();
    A.block<3, 1>(3, 0) = Sx_bar;

    A.block<1, 3>(1, 6) = Sx_bar.transpose();
    A.block<3, 1>(6, 1) = Sx_bar;

    A.block<1, 3>(2, 9) = Sx_bar.transpose();
    A.block<3, 1>(9, 2) = Sx_bar;

    A.block<3, 3>(3, 3) = Sx_bar_x_bar;
    A.block<3, 3>(6, 6) = Sx_bar_x_bar;
    A.block<3, 3>(9, 9) = Sx_bar_x_bar;

    Vector12  q           = A.ldlt().solve(SJx);
    Matrix4x4 ret         = Matrix4x4::Identity();
    ret.block<1, 3>(0, 0) = q.segment<3>(3).transpose();
    ret.block<1, 3>(1, 0) = q.segment<3>(6).transpose();
    ret.block<1, 3>(2, 0) = q.segment<3>(9).transpose();
    ret.block<3, 1>(0, 3) = q.segment<3>(0);

    return ret;
}

Matrix4x4 optimal_transform(span<const Vector3> S, span<const Vector3> D)
{
    return optimal_transform(Matrix4x4::Identity(), S, Matrix4x4::Identity(), D);
}

Matrix4x4 optimal_transform(const SimplicialComplex& S, const SimplicialComplex& D)
{
    UIPC_ASSERT(S.vertices().size() >= 4,
                "The number of points({}) should be at least 4.",
                S.vertices().size());
    UIPC_ASSERT(S.vertices().size() == D.vertices().size(),
                "The number of source points({}) and destination points({}) should be the same.",
                S.vertices().size(),
                D.vertices().size());
    UIPC_ASSERT(S.instances().size() == 1,
                "The number of instances({}) should be 1.",
                S.instances().size());
    UIPC_ASSERT(D.instances().size() == 1,
                "The number of instances({}) should be 1.",
                D.instances().size());

    auto SV = S.positions().view();
    auto DV = D.positions().view();
    auto ST = S.transforms().view()[0];
    auto DT = D.transforms().view()[0];

    return optimal_transform(ST, SV, DT, DV);
}
}  // namespace uipc::geometry