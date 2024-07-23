
// > Squared Version
// > D := d*d 
// > DHat := dHat*dHat

template <typename T>
__host__ __device__ void KappaBarrier(T& R, const T& kappa, const T& D, const T& DHat)
{
/*****************************************************************************************************************************
Function generated by SymEigen.py 
Author: MuGdxy
GitHub: https://github.com/MuGdxy/SymEigen
E-Mail: lxy819469559@gmail.com
******************************************************************************************************************************
LaTeX expression:
//tex:$$R = - \kappa \left(D - \hat{D}\right)^{2} \log{\left(\frac{D}{\hat{D}} \right)}$$

Symbol Name Mapping:
kappa:
    -> {}
    -> Matrix([[kappa]])
D:
    -> {}
    -> Matrix([[D]])
DHat:
    -> {}
    -> Matrix([[DHat]])
*****************************************************************************************************************************/
/* Sub Exprs */
/* Simplified Expr */
R = -kappa*((D - DHat) * (D - DHat))*log(D/DHat);
}
template <typename T>
__host__ __device__ void dKappaBarrierdD(T& R, const T& kappa, const T& D, const T& DHat)
{
/*****************************************************************************************************************************
Function generated by SymEigen.py 
Author: MuGdxy
GitHub: https://github.com/MuGdxy/SymEigen
E-Mail: lxy819469559@gmail.com
******************************************************************************************************************************
LaTeX expression:
//tex:$$R = - \kappa \left(2 D - 2 \hat{D}\right) \log{\left(\frac{D}{\hat{D}} \right)} - \frac{\kappa \left(D - \hat{D}\right)^{2}}{D}$$

Symbol Name Mapping:
kappa:
    -> {}
    -> Matrix([[kappa]])
D:
    -> {}
    -> Matrix([[D]])
DHat:
    -> {}
    -> Matrix([[DHat]])
*****************************************************************************************************************************/
/* Sub Exprs */
/* Simplified Expr */
R = -kappa*(2*D - 2*DHat)*log(D/DHat) - kappa*((D - DHat) * (D - DHat))/D;
}
template <typename T>
__host__ __device__ void ddKappaBarrierddD(T& R, const T& kappa, const T& D, const T& DHat)
{
/*****************************************************************************************************************************
Function generated by SymEigen.py 
Author: MuGdxy
GitHub: https://github.com/MuGdxy/SymEigen
E-Mail: lxy819469559@gmail.com
******************************************************************************************************************************
LaTeX expression:
//tex:$$R = - 2 \kappa \log{\left(\frac{D}{\hat{D}} \right)} - \frac{2 \kappa \left(2 D - 2 \hat{D}\right)}{D} + \frac{\kappa \left(D - \hat{D}\right)^{2}}{D^{2}}$$

Symbol Name Mapping:
kappa:
    -> {}
    -> Matrix([[kappa]])
D:
    -> {}
    -> Matrix([[D]])
DHat:
    -> {}
    -> Matrix([[DHat]])
*****************************************************************************************************************************/
/* Sub Exprs */
auto x0 = 2*kappa;
/* Simplified Expr */
R = -x0*log(D/DHat) - x0*(2*D - 2*DHat)/D + kappa*((D - DHat) * (D - DHat))/((D) * (D));
}
/*
    __device__ Float smooth_function(Float eps_v, 
                                     Float h_hat, 
                                     Float y) 
    {
        Float scalar = eps_v * h_hat;
        if(0 < y && y < scalar)
        {
            return -y * y * y / (3 * scalar * scalar) + y * y / scalar + scalar / 3;
        }
    }
*/
template <typename T>
__host__ __device__ void FrictionEnergy(T& R, const T& coefficient, const T& eps_v, const T& h_hat, const T& y)
{
    T scalar = eps_v * h_hat;
    if (0 < y && y < scalar) {
        R = coefficient * (-y * y * y / (3 * scalar * scalar) + y * y / scalar + scalar / 3);
    } else if (y >= scalar) {
        R = coefficient * y;
    } else {
        R = 0;
    }
}

template <typename T>
__host__ __device__ void dFrictionEnergydV(Eigen::Vector<T, 6>& R, const T& coefficient, const Eigen::Matrix<T, 6, 3>& Tk, const T& eps_v, const T& h_hat, const Eigen::Vector<T, 3>& vk)
{
    T y = vk.norm();
    Eigen::Vector<T, 3> s = vk / y; 
    if (0 < y && y < eps_v) {
        T f = - y * y / (eps_v * eps_v) + 2 * y / eps_v;
        R = coefficient * Tk * f * s * h_hat;
    }
    else if (y >= eps_v) {
        R = coefficient * Tk * s * h_hat;
    } else {
        R = Eigen::Vector<T, 6>::Zero();
    }
}

template <typename T>
__host__ __device__ void ddFrictionEnergyddV(Eigen::Matrix<T, 6, 6>& R, const T& coefficient, const Eigen::Matrix<T, 6, 3>& Tk, const T& eps_v, const T& h_hat, const Eigen::Vector<T, 3>& vk)
{
    T scalar = eps_v * h_hat;
    T y = vk.norm();
    Eigen::Vector<T, 3> s = vk / y; 
    if (0 < y && y < eps_v) {
        T f = - 2 / (eps_v * eps_v) + 2 / eps_v;
        Eigen::Matrix<T, 3, 3> M = -vk * vk.transpose() / (eps_v * eps_v * y) + Eigen::Matrix<T, 3, 3>::Identity() * f / eps_v;
        R = coefficient * Tk * M * Tk.transpose() * h_hat;
    } else if (y >= eps_v) {
        T f = - 2 / (eps_v * eps_v) + 2 / eps_v;
        Eigen::Matrix<T, 3, 3> M = -vk * vk.transpose() / (y * y * y) + Eigen::Matrix<T, 3, 3>::Identity() * f / eps_v;
        R = coefficient * Tk * M * Tk.transpose() * h_hat;
    } else {
        R = Eigen::Matrix<T, 6, 6>::Zero();
    }
}