#include <finite_element/fem_utils.h>
#include <muda/ext/eigen/inverse.h>
#include <Eigen/Geometry>

namespace uipc::backend::cuda::fem
{
UIPC_GENERIC Float invariant2(const Matrix3x3& F)
{
    return ddot(F, F);
}

UIPC_GENERIC Float invariant2(const Vector3& Sigma)
{
    return Sigma[0] * Sigma[0] + Sigma[1] * Sigma[1] + Sigma[2] * Sigma[2];
}

UIPC_GENERIC Float invariant3(const Matrix3x3& F)
{
    return F.determinant();
}

UIPC_GENERIC Float invariant3(const Vector3& Sigma)
{
    return Sigma[0] * Sigma[1] * Sigma[2];
}

UIPC_GENERIC Float invariant4(const Matrix3x3& F, const Vector3& a)
{
    Matrix3x3 U, V;
    Vector3   Sigma;
    svd(F, U, Sigma, V);
    const Matrix3x3 S = V * Sigma.asDiagonal() * V.transpose();
    return (S * a).dot(a);
}

UIPC_GENERIC Float invariant5(const Matrix3x3& F, const Vector3& a)
{
    return (F * a).squaredNorm();
}

UIPC_GENERIC Matrix3x3 dJdF(const Matrix3x3& F)
{
    Matrix3x3 dJdF;
    //tex:
    //$$
    //\frac{\partial I_{3}}{\partial \mathbf{F}}=\frac{\partial J}{\partial \mathbf{F}}=\left[\begin{array}{l|l|l}
    //\mathbf{f}_{1} \times \mathbf{f}_{2} & \mathbf{f}_{2} \times \mathbf{f}_{0} & \mathbf{f}_{0} \times \mathbf{f}_{1}
    //\end{array}\right]
    //$$
    dJdF.col(0) = F.col(1).cross(F.col(2));
    dJdF.col(1) = F.col(2).cross(F.col(0));
    dJdF.col(2) = F.col(0).cross(F.col(1));
    return dJdF;
}

UIPC_GENERIC Matrix3x3 Dm_inv(const Vector3& X0, const Vector3& X1, const Vector3& X2, const Vector3& X3)
{
    Matrix3x3 Dm = Ds(X0, X1, X2, X3);
    return muda::eigen::inverse(Dm);
}

UIPC_GENERIC Matrix3x3 Ds(const Vector3& x0, const Vector3& x1, const Vector3& x2, const Vector3& x3)
{
    Matrix3x3 Ds;
    Ds.col(0) = x1 - x0;
    Ds.col(1) = x2 - x0;
    Ds.col(2) = x3 - x0;
    return Ds;
}

UIPC_GENERIC Matrix9x12 dFdx(const Matrix3x3& DmInv)
{
    // clang-format off
    
    //tex:
    //$$
    //\begin{array}{l}
    //\frac{\partial \mathbf{D}_{s}}{\partial x_{0}}=\left[\begin{array}{ccc}
    //-1 & -1 & -1 \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{1}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //-1 & -1 & -1 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{2}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //-1 & -1 & -1
    //\end{array}\right] \\
    //\frac{\partial \mathbf{D}_{s}}{\partial x_{3}}=\left[\begin{array}{ccc}
    //1 & 0 & 0 \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{4}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //1 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{5}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //1 & 0 & 0
    //\end{array}\right] \\
    //\frac{\partial \mathbf{D}_{s}}{\partial x_{6}}=\left[\begin{array}{ccc}
    //0 & 1 & 0 \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{7}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 1 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{8}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //0 & 1 & 0
    //\end{array}\right] \\
    //\frac{\partial \mathbf{D}_{s}}{\partial x_{9}}=\left[\begin{array}{ccc}
    //0 & 0 & 1 \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{10}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 1 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{D}_{s}}{\partial x_{11}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //0 & 0 & 1
    //\end{array}\right] \\
    //\end{array}
    //$$
    

    //tex:
    //$$ \mathbf{D}_m^{-1}=\left[\begin{array}{l}
    //\mathbf{r}_{0} \\
    //\hline \mathbf{r}_{1} \\
    //\hline \mathbf{r}_{2}
    //\end{array}\right]
    //=
    //\left[\begin{array}{l|l|l} 
    //\mathbf{c}_{0} & \mathbf{c}_{1} & \mathbf{c}_{2}
    //\end{array}\right]
	//$$


    //tex:
    //$$
    //\begin{array}{l}
    //\frac{\partial \mathbf{F}}{\partial x_{0}}=\left[\begin{array}{ccc}
    //-s_{0} & -s_{1} & -s_{2} \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{1}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //-s_{0} & -s_{1} & -s_{2} \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{2}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //-s_{0} & -s_{1} & -s_{2}
    //\end{array}\right] \\
    //\frac{\partial \mathbf{F}}{\partial x_{3}}=\left[\begin{array}{ccc} 
    //& \mathbf{r}_{0} \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{4}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //& \mathbf{r}_{0} & \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{5}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //& \mathbf{r}_{0} &
    //\end{array}\right] \\
    //\frac{\partial \mathbf{F}}{\partial x_{6}}=\left[\begin{array}{ccc} 
    //& \mathbf{r}_{1} \\
    //0 & 0 & 0 \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{7}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //& \mathbf{r}_{1} & \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{8}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //& \mathbf{r}_{1} &
    //\end{array}\right] \\
    //\frac{\partial \mathbf{F}}{\partial x_{9}}=\left[\begin{array}{ccc} 
    //& \mathbf{r}_{2} & 0 \\
    //0 & 0 & \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{10}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //& \mathbf{r}_{2} & \\
    //0 & 0 & 0
    //\end{array}\right] \quad \frac{\partial \mathbf{F}}{\partial x_{11}}=\left[\begin{array}{ccc}
    //0 & 0 & 0 \\
    //0 & 0 & 0 \\
    //& \mathbf{r}_{2} &
    //\end{array}\right] \\
    //\end{array}
    //$$


    // clang-format on
    const Float m = DmInv(0, 0);
    const Float n = DmInv(0, 1);
    const Float o = DmInv(0, 2);
    const Float p = DmInv(1, 0);
    const Float q = DmInv(1, 1);
    const Float r = DmInv(1, 2);
    const Float s = DmInv(2, 0);
    const Float t = DmInv(2, 1);
    const Float u = DmInv(2, 2);

    const Float t1 = -m - p - s;
    const Float t2 = -n - q - t;
    const Float t3 = -o - r - u;

    Matrix9x12 PFPu = Matrix9x12::Zero();
    PFPu(0, 0)      = t1;
    PFPu(0, 3)      = m;
    PFPu(0, 6)      = p;
    PFPu(0, 9)      = s;
    PFPu(1, 1)      = t1;
    PFPu(1, 4)      = m;
    PFPu(1, 7)      = p;
    PFPu(1, 10)     = s;
    PFPu(2, 2)      = t1;
    PFPu(2, 5)      = m;
    PFPu(2, 8)      = p;
    PFPu(2, 11)     = s;
    PFPu(3, 0)      = t2;
    PFPu(3, 3)      = n;
    PFPu(3, 6)      = q;
    PFPu(3, 9)      = t;
    PFPu(4, 1)      = t2;
    PFPu(4, 4)      = n;
    PFPu(4, 7)      = q;
    PFPu(4, 10)     = t;
    PFPu(5, 2)      = t2;
    PFPu(5, 5)      = n;
    PFPu(5, 8)      = q;
    PFPu(5, 11)     = t;
    PFPu(6, 0)      = t3;
    PFPu(6, 3)      = o;
    PFPu(6, 6)      = r;
    PFPu(6, 9)      = u;
    PFPu(7, 1)      = t3;
    PFPu(7, 4)      = o;
    PFPu(7, 7)      = r;
    PFPu(7, 10)     = u;
    PFPu(8, 2)      = t3;
    PFPu(8, 5)      = o;
    PFPu(8, 8)      = r;
    PFPu(8, 11)     = u;

    return PFPu;
}

UIPC_GENERIC Matrix3x3 F(const Vector3&   x0,
                         const Vector3&   x1,
                         const Vector3&   x2,
                         const Vector3&   x3,
                         const Matrix3x3& DmInv)
{
    auto ds = Ds(x0, x1, x2, x3);
    return ds * DmInv;
}
}  // namespace uipc::backend::cuda
