#include <affine_body/abd_jacobi_matrix.h>

namespace uipc::backend::cuda
{
MUDA_GENERIC Vector12 operator*(const ABDJacobi::ABDJacobiT& j, const Vector3& g)
{
    Vector12    g12;
    const auto& x     = j.m_j.m_x_bar;
    g12.segment<3>(0) = g;
    g12.segment<3>(3) = x * g.x();
    g12.segment<3>(6) = x * g.y();
    g12.segment<3>(9) = x * g.z();
    return g12;
}
MUDA_GENERIC Vector3 operator*(const ABDJacobi& j, const Vector12& q)
{
    const auto& t  = q.segment<3>(0);
    const auto& a1 = q.segment<3>(3);
    const auto& a2 = q.segment<3>(6);
    const auto& a3 = q.segment<3>(9);
    const auto& x  = j.m_x_bar;
    return Vector3{x.dot(a1), x.dot(a2), x.dot(a3)} + t;
}

//tex:
//$$
//\left[\begin{array}{ccc|ccc:ccc:ccc}
//1 &   &   & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  &  &  & \\
//& 1 &   &  &  &  & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  \\
//&   & 1 &  &  &  &  &  &  &  \bar{x}_1 & \bar{x}_2 & \bar{x}_3\\
//\end{array}\right]
//$$
MUDA_GENERIC Matrix3x12 ABDJacobi::to_mat() const
{
    Matrix3x12  ret       = Matrix3x12::Zero();
    const auto& x         = m_x_bar;
    ret(0, 0)             = 1;
    ret(1, 1)             = 1;
    ret(2, 2)             = 1;
    ret.block<1, 3>(0, 3) = x.transpose();
    ret.block<1, 3>(1, 6) = x.transpose();
    ret.block<1, 3>(2, 9) = x.transpose();
    return ret;
}

MUDA_GENERIC Matrix12x12 ABDJacobi::JT_H_J(const ABDJacobiT& lhs_J_T,
                                           const Matrix3x3&  Hessian,
                                           const ABDJacobi&  rhs_J)
{
    //tex:
    //$$
    //\begin{bmatrix}
    //\mathbf{H} & \mathbf{c}_1\cdot\bar{\mathbf{y}}^{T} & \mathbf{c}_2\cdot\bar{\mathbf{y}}^{T} & \mathbf{c}_3\cdot\bar{\mathbf{y}}^{T}\\
    //\bar{\mathbf{x}}\cdot\mathbf{r}_1 & H_{11}\cdot\bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T}  & H_{12}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T} & H_{13}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T}\\
    //\bar{\mathbf{x}}\cdot\mathbf{r}_2 & H_{21}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T} & H_{22}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T} & H_{23}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T}\\
    //\bar{\mathbf{x}}\cdot\mathbf{r}_3 & H_{31}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T} & H_{32}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T} & H_{33}\cdot \bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T}\\
    //\end{bmatrix}
    //$$

    Matrix12x12 ret       = Matrix12x12::Zero();
    auto        x         = lhs_J_T.J().x_bar();
    auto        y         = rhs_J.x_bar();
    ret.block<3, 3>(0, 0) = Hessian;

    // Hessian Col * y
    ret.block<3, 3>(0, 3) = Hessian.block<3, 1>(0, 0) * y.transpose();
    ret.block<3, 3>(0, 6) = Hessian.block<3, 1>(0, 1) * y.transpose();
    ret.block<3, 3>(0, 9) = Hessian.block<3, 1>(0, 2) * y.transpose();

    // x * Hessian Row
    ret.block<3, 3>(3, 0) = x * Hessian.block<1, 3>(0, 0);
    ret.block<3, 3>(6, 0) = x * Hessian.block<1, 3>(1, 0);
    ret.block<3, 3>(9, 0) = x * Hessian.block<1, 3>(2, 0);

    Matrix3x3 x_y = x * y.transpose();

    // kronecker product
    //tex: $$ \mathbf{H} \otimes (\bar{\mathbf{x}}\cdot\bar{\mathbf{y}}^{T})$$
    ret.block<3, 3>(3, 3) = x_y * Hessian(0, 0);
    ret.block<3, 3>(3, 6) = x_y * Hessian(0, 1);
    ret.block<3, 3>(3, 9) = x_y * Hessian(0, 2);

    ret.block<3, 3>(6, 3) = x_y * Hessian(1, 0);
    ret.block<3, 3>(6, 6) = x_y * Hessian(1, 1);
    ret.block<3, 3>(6, 9) = x_y * Hessian(1, 2);

    ret.block<3, 3>(9, 3) = x_y * Hessian(2, 0);
    ret.block<3, 3>(9, 6) = x_y * Hessian(2, 1);
    ret.block<3, 3>(9, 9) = x_y * Hessian(2, 2);

    return ret;
}

MUDA_GENERIC Vector12 operator*(const ABDJacobiDyadicMass& JTJ, const Vector12& p)
{
    Vector12    ret;
    const auto& m = JTJ.m_mass;
    const auto& D = JTJ.m_mass_times_dyadic_x_bar;
    const auto& x = JTJ.m_mass_times_x_bar;

    const auto& p_p  = p.segment<3>(0);
    const auto& p_a1 = p.segment<3>(3);
    const auto& p_a2 = p.segment<3>(6);
    const auto& p_a3 = p.segment<3>(9);

    //tex:
    //$$
    //\left[\begin{array}{c}
    //\mathbf{p}_{\mathbf{a}} \mathbf{x} + m\mathbf{p}_{\mathbf{p}}\\
    //\mathbf{D} \cdot \mathbf{p}_{\mathbf{a}_1} + \mathbf{x} \cdot p_1\\
    //\mathbf{D} \cdot \mathbf{p}_{\mathbf{a}_2} + \mathbf{x} \cdot p_2\\
    //\mathbf{D} \cdot \mathbf{p}_{\mathbf{a}_3} + \mathbf{x} \cdot p_3\\
    //\end{array}\right]_{12\times1}
    //$$

    ret(0) = x.dot(p_a1) + m * p_p.x();
    ret(1) = x.dot(p_a2) + m * p_p.y();
    ret(2) = x.dot(p_a3) + m * p_p.z();

    ret.segment<3>(3) = D * p_a1 + x * p_p.x();
    ret.segment<3>(6) = D * p_a2 + x * p_p.y();
    ret.segment<3>(9) = D * p_a3 + x * p_p.z();

    return ret;
}

MUDA_GENERIC ABDJacobiDyadicMass& ABDJacobiDyadicMass::operator+=(const ABDJacobiDyadicMass& rhs)
{
    m_mass += rhs.m_mass;
    m_mass_times_x_bar += rhs.m_mass_times_x_bar;
    m_mass_times_dyadic_x_bar += rhs.m_mass_times_dyadic_x_bar;
    return *this;
}
//tex:
//$$
//\mathbf{J}^T\mathbf{J}
//= \left[\begin{array}{cccccccccccc}
//1 & 0 & 0 & \bar{x}_{1} & \bar{x}_{2} & \bar{x}_{3} & 0 & 0 & 0 & 0 & 0 & 0\\
//0 & 1 & 0 & 0 & 0 & 0 & \bar{x}_{1} & \bar{x}_{2} & \bar{x}_{3} & 0 & 0 & 0\\
//0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & \bar{x}_{1} & \bar{x}_{2} & \bar{x}_{3}\\
//\bar{x}_{1} & 0 & 0 & \bar{x}_{1}^{2} & \bar{x}_{1} \bar{x}_{2} & \bar{x}_{1} \bar{x}_{3} & 0 & 0 & 0 & 0 & 0 & 0\\
//\bar{x}_{2} & 0 & 0 & \bar{x}_{1} \bar{x}_{2} & \bar{x}_{2}^{2} & \bar{x}_{2} \bar{x}_{3} & 0 & 0 & 0 & 0 & 0 & 0\\
//\bar{x}_{3} & 0 & 0 & \bar{x}_{1} \bar{x}_{3} & \bar{x}_{2} \bar{x}_{3} & \bar{x}_{3}^{2} & 0 & 0 & 0 & 0 & 0 & 0\\
//0 & \bar{x}_{1} & 0 & 0 & 0 & 0 & \bar{x}_{1}^{2} & \bar{x}_{1} \bar{x}_{2} & \bar{x}_{1} \bar{x}_{3} & 0 & 0 & 0\\
//0 & \bar{x}_{2} & 0 & 0 & 0 & 0 & \bar{x}_{1} \bar{x}_{2} & \bar{x}_{2}^{2} & \bar{x}_{2} \bar{x}_{3} & 0 & 0 & 0\\
//0 & \bar{x}_{3} & 0 & 0 & 0 & 0 & \bar{x}_{1} \bar{x}_{3} & \bar{x}_{2} \bar{x}_{3} & \bar{x}_{3}^{2} & 0 & 0 & 0\\
//0 & 0 & \bar{x}_{1} & 0 & 0 & 0 & 0 & 0 & 0 & \bar{x}_{1}^{2} & \bar{x}_{1} \bar{x}_{2} & \bar{x}_{1} \bar{x}_{3}\\
//0 & 0 & \bar{x}_{2} & 0 & 0 & 0 & 0 & 0 & 0 & \bar{x}_{1} \bar{x}_{2} & \bar{x}_{2}^{2} & \bar{x}_{2} \bar{x}_{3}\\
//0 & 0 & \bar{x}_{3} & 0 & 0 & 0 & 0 & 0 & 0 & \bar{x}_{1} \bar{x}_{3} & \bar{x}_{2} \bar{x}_{3} & \bar{x}_{3}^{2}\end{array}\right]
//$$
MUDA_GENERIC void ABDJacobiDyadicMass::add_to(Matrix12x12& h) const
{
    // row 0 + col 0
    h(0, 0) += m_mass;
    h.block<1, 3>(0, 3) += m_mass_times_x_bar.transpose();
    h.block<3, 1>(3, 0) += m_mass_times_x_bar;

    // row 1 + col 1
    h(1, 1) += m_mass;
    h.block<1, 3>(1, 6) += m_mass_times_x_bar.transpose();
    h.block<3, 1>(6, 1) += m_mass_times_x_bar;

    // row 2 + col 2
    h(2, 2) += m_mass;
    h.block<1, 3>(2, 9) += m_mass_times_x_bar.transpose();
    h.block<3, 1>(9, 2) += m_mass_times_x_bar;

    // block<3,3> at (3,3)  (6,6)  (9,9)
    h.block<3, 3>(3, 3) += m_mass_times_dyadic_x_bar;
    h.block<3, 3>(6, 6) += m_mass_times_dyadic_x_bar;
    h.block<3, 3>(9, 9) += m_mass_times_dyadic_x_bar;
}
MUDA_GENERIC Matrix12x12 ABDJacobiDyadicMass::to_mat() const
{
    Matrix12x12 h = Matrix12x12::Zero();
    add_to(h);
    return h;
}

MUDA_DEVICE ABDJacobiDyadicMass ABDJacobiDyadicMass::atomic_add(ABDJacobiDyadicMass& dst,
                                                                const ABDJacobiDyadicMass& src)
{
    ABDJacobiDyadicMass ret;
    auto                mass = muda::atomic_add(&dst.m_mass, src.m_mass);
    auto                mass_times_x_bar =
        muda::eigen::atomic_add(dst.m_mass_times_x_bar, src.m_mass_times_x_bar);
    auto mass_times_dyadic_x_bar =
        muda::eigen::atomic_add(dst.m_mass_times_dyadic_x_bar, src.m_mass_times_dyadic_x_bar);
    ret.m_mass                    = mass;
    ret.m_mass_times_x_bar        = mass_times_x_bar;
    ret.m_mass_times_dyadic_x_bar = mass_times_dyadic_x_bar;
    return ret;
}
}  // namespace uipc::backend::cuda
