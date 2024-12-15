#pragma once
#include <affine_body/type_define.h>
#include <cuda_runtime_api.h>
#include <muda/muda_def.h>
namespace uipc::backend::cuda
{
//tex: $$ \mathbf{J}_{3 \times 12} $$ or $$ (\mathbf{J}^T)_{12 \times 3} $$
class ABDJacobi  // for every point
{
  public:
    class ABDJacobiT
    {
        const ABDJacobi& m_j;

      public:
        explicit MUDA_GENERIC ABDJacobiT(const ABDJacobi& j)
            : m_j(j)
        {
        }
        MUDA_GENERIC friend Vector12 operator*(const ABDJacobiT& j, const Vector3& g);

        MUDA_GENERIC const auto& J() const { return m_j; }
    };
    MUDA_GENERIC ABDJacobi(const Vector3& x_bar)
        : m_x_bar(x_bar)
    {
    }

    MUDA_GENERIC ABDJacobi()
        : m_x_bar(Vector3::Zero())
    {
    }

    MUDA_GENERIC friend Vector3 operator*(const ABDJacobi& j, const Vector12& q);
    MUDA_GENERIC friend Vector12 operator*(const ABDJacobi::ABDJacobiT& j, const Vector3& g);

    MUDA_GENERIC Vector3 point_from_affine(const Vector12& q)
    {
        return (*this) * q;
    }

    MUDA_GENERIC Vector3 point_x(const Vector12& q) const
    {
        return (*this) * q;
    };

    MUDA_GENERIC Matrix3x12 to_mat() const;

    MUDA_GENERIC ABDJacobiT T() const { return ABDJacobiT(*this); }

    MUDA_GENERIC const Vector3& x_bar() const { return m_x_bar; }

    //tex: $$ \mathbf{J}^T\mathbf{H}\mathbf{J} $$
    static MUDA_GENERIC Matrix12x12 JT_H_J(const ABDJacobiT& lhs_J_T,
                                           const Matrix3x3&  Hessian,
                                           const ABDJacobi&  rhs_J);

  private:
    //tex: $$ \bar{\mathbf{x}} $$
    Vector3 m_x_bar;
};

template <size_t N>
class ABDJacobiStack
{
  protected:
    const ABDJacobi* m_jacobis[N];

  public:
    class ABDJacobiStackT
    {
        const ABDJacobiStack& m_origin;

      public:
        MUDA_GENERIC ABDJacobiStackT(const ABDJacobiStack& j)
            : m_origin(j)
        {
        }
        MUDA_GENERIC Vector12 operator*(const Vector<Float, 3 * N>& g) const;
    };

    MUDA_GENERIC Vector<Float, 3 * N> operator*(const Vector12& q) const;

    MUDA_GENERIC Matrix<Float, 3 * N, 12> to_mat() const;

    MUDA_GENERIC ABDJacobiStackT T() const { return ABDJacobiStackT(*this); }
};

class ABDJacobiStack2 : public ABDJacobiStack<2>
{
  public:
    MUDA_GENERIC ABDJacobiStack2(const ABDJacobi& j1, const ABDJacobi& j2)
    {
        m_jacobis[0] = &j1;
        m_jacobis[1] = &j2;
    }
};

class ABDJacobiStack3 : public ABDJacobiStack<3>
{
  public:
    MUDA_GENERIC ABDJacobiStack3(const ABDJacobi& j1, const ABDJacobi& j2, const ABDJacobi& j3)
    {
        m_jacobis[0] = &j1;
        m_jacobis[1] = &j2;
        m_jacobis[2] = &j3;
    }
};


//tex:
// $$
//\mathbf{g}^{\text{Affine}}_k = \sum_{i\in \mathscr{C}_k \cap \mathscr{A}}
//\mathbf{J}_i^T \frac{\partial B}{\partial\mathbf{x}_i}
//= \sum_{i\in \mathscr{C}_k \cap \mathscr{A}}
//
//\begin{bmatrix}
//g_{1}\\
//g_{2}\\
//g_{3}\\
//\hline
//
//\bar{x}_1 g_{1}\\
//\bar{x}_2 g_{1}\\
//\bar{x}_3 g_{1}\\
//\hdashline
//
//\bar{x}_1 g_{2}\\
//\bar{x}_2 g_{2}\\
//\bar{x}_3 g_{2}\\
//\hdashline
//
//\bar{x}_1 g_{3}\\
//\bar{x}_2 g_{3}\\
//\bar{x}_3 g_{3}
//
//\end{bmatrix}_{i}
//
//=
//\sum_{i\in \mathscr{C}_k \cap \mathscr{A}}
//
//\begin{bmatrix}
//\mathbf{g}\\
//\hline
//
//g_{1} \bar{\mathbf{x}}\\
//\hdashline
//
//g_{2} \bar{\mathbf{x}}\\
//\hdashline
//
//g_{3} \bar{\mathbf{x}}\\
//
//\end{bmatrix}_{i}
// $$

//tex:
// where $\mathscr{C}_k$ is the $k$-th contact pair, and $\mathscr{A}$ represents the point set of all affine bodies.
//

//tex: $$\mathbf{J}^T\mathbf{M}_i\mathbf{J} $$
class ABDJacobiDyadicMass
{
  public:
    MUDA_GENERIC ABDJacobiDyadicMass()
        : m_mass(0)
        , m_mass_times_x_bar(Vector3::Zero())
        , m_mass_times_dyadic_x_bar(Matrix3x3::Zero())
    {
    }

    MUDA_GENERIC static ABDJacobiDyadicMass from_dyadic_mass(Float sum_m,
                                                             const Vector3& sum_m_x_bar,
                                                             const Matrix3x3& sum_m_x_bar_x_bar)
    {
        ABDJacobiDyadicMass ret;
        ret.m_mass                    = sum_m;
        ret.m_mass_times_x_bar        = sum_m_x_bar;
        ret.m_mass_times_dyadic_x_bar = sum_m_x_bar_x_bar;
        return ret;
    }

    MUDA_GENERIC ABDJacobiDyadicMass(double node_mass, const Vector3& x_bar)
        : m_mass(node_mass)
        , m_mass_times_x_bar(node_mass * x_bar)
        , m_mass_times_dyadic_x_bar((node_mass * x_bar) * x_bar.transpose())
    {
    }

    MUDA_GENERIC friend Vector12 operator*(const ABDJacobiDyadicMass& mJTJ,
                                           const Vector12&            p);

    MUDA_GENERIC ABDJacobiDyadicMass& operator+=(const ABDJacobiDyadicMass& rhs);

    MUDA_GENERIC void add_to(Matrix12x12& h) const;

    MUDA_GENERIC Matrix12x12 to_mat() const;

    MUDA_GENERIC double mass() const { return m_mass; }

    static MUDA_GENERIC auto zero() { return ABDJacobiDyadicMass{}; }

    static MUDA_DEVICE ABDJacobiDyadicMass atomic_add(ABDJacobiDyadicMass& dst,
                                                      const ABDJacobiDyadicMass& src);

  private:
    double m_mass;
    //tex: $$ m\bar{\mathbf{x}} $$
    Vector3 m_mass_times_x_bar;
    //tex: $$ m\bar{\mathbf{x}} \otimes \bar{\mathbf{x}} $$
    Matrix3x3 m_mass_times_dyadic_x_bar;
};
}  // namespace uipc::backend::cuda

#include "details/abd_jacobi_matrix.inl"