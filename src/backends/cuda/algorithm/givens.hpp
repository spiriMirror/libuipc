#pragma once
#include <cmath>
#include <type_traits>
#include <Eigen/Eigen>

namespace uipc::backend::cuda
{
namespace math
{

    template <typename T>
    struct GivensRotation
    {
        // act on indices rowi and rowk (assume rowi < rowk)
        int rowi{0};
        int rowk{1};
        T   c{T(1)};
        T   s{T(0)};

        constexpr GivensRotation() noexcept = default;
        constexpr GivensRotation(int rowi_in, int rowk_in) noexcept
            : rowi{rowi_in}
            , rowk{rowk_in}
            , c{T(1)}
            , s{T(0)}
        {
        }

        constexpr GivensRotation(T a, T b, int rowi_in, int rowk_in) noexcept
            : rowi{rowi_in}
            , rowk{rowk_in}
        {
            computeConventional(a, b);
        }

        constexpr void setIdentity() noexcept
        {
            c = T(1);
            s = T(0);
        }

        // transpose of 2D rotation flips sign of s
        constexpr void transposeInPlace() noexcept { s = -s; }

        // Conventional: [c -s; s c] * [a; b] = [*; 0]
        constexpr void computeConventional(const T a, const T b)
        {
            const T d = a * a + b * b;
            c         = T(1);
            s         = T(0);
            if(d != T(0))
            {
                const T t = T(1) / std::sqrt(d);
                c         = a * t;
                s         = -b * t;
            }
        }

        // Unconventional: [c -s; s c] * [a; b] = [0; *]
        constexpr void computeUnconventional(const T a, const T b)
        {
            const T d = a * a + b * b;
            c         = T(0);
            s         = T(1);
            if(d != T(0))
            {
                const T t = T(1) / std::sqrt(d);
                s         = a * t;
                c         = b * t;
            }
        }

        // Fill an identity and insert this 2x2 rotation into (rowi,rowk) block
        template <typename Mat>
        constexpr void fill(Eigen::MatrixBase<Mat>& R) const
        {
            //static_assert(Mat::RowsAtCompileTime == Mat::ColsAtCompileTime,
            //              "square matrix required");
            R.derived().setIdentity();
            R(rowi, rowi) = c;
            R(rowk, rowi) = -s;
            R(rowi, rowk) = s;
            R(rowk, rowk) = c;
        }

        // Row rotation: A <- G^T A; acts on rows rowi,rowk
        template <typename Mat>
        constexpr void rowRotation(Eigen::MatrixBase<Mat>& A) const
        {
            //using std::swap;
            const int ncols = int(A.cols());
            //printf("%d\n", ncols);
            for(int j = 0; j < ncols; ++j)
            {
                const T tau1 = A(rowi, j);
                const T tau2 = A(rowk, j);
                A(rowi, j)   = c * tau1 - s * tau2;
                A(rowk, j)   = s * tau1 + c * tau2;
            }
        }

        // Column rotation: A <- A G; acts on cols rowi,rowk
        template <typename Mat>
        constexpr void columnRotation(Eigen::MatrixBase<Mat>& A) const
        {
            const int nrows = int(A.rows());
            for(int i = 0; i < nrows; ++i)
            {
                const T tau1 = A(i, rowi);
                const T tau2 = A(i, rowk);
                A(i, rowi)   = c * tau1 - s * tau2;
                A(i, rowk)   = s * tau1 + c * tau2;
            }
        }

        // Compose rotations acting on the same pair (rowi,rowk)
        constexpr void operator*=(const GivensRotation<T>& R) noexcept
        {
            const T new_c = c * R.c - s * R.s;
            const T new_s = s * R.c + c * R.s;
            c             = new_c;
            s             = new_s;
        }

        constexpr GivensRotation<T> operator*(const GivensRotation<T>& R) const noexcept
        {
            GivensRotation<T> out{*this};
            out *= R;
            return out;
        }
    };

    /**
 * Zero chasing for 3x3 to make upper-bidiagonal (follow ziran2020 style)
 * Input H assumed of the form:
 *   x x 0
 *   x x x
 *   0 0 x
 * After:
 *   x x 0
 *   0 x x
 *   0 0 x
 * U and V are accumulated so that U * H_original * V^T = H_bidiag
 */
    template <typename MatH, typename MatU, typename MatV>
    constexpr void zero_chasing(Eigen::MatrixBase<MatH>& H,
                                Eigen::MatrixBase<MatU>& U,
                                Eigen::MatrixBase<MatV>& V)
    {
        //static_assert(is_3_by_3_eigen<MatH>(), "H must be 3x3");
        //static_assert(is_3_by_3_eigen<MatU>(), "U must be 3x3");
        //static_assert(is_3_by_3_eigen<MatV>(), "V must be 3x3");
        using T = typename MatH::Scalar;

        // r1: act on rows (0,1) to zero H(1,0) after rotation (classic step)
        GivensRotation<T> r1(H(0, 0), H(1, 0), 0, 1);

        // r2: act on cols (1,2) to zero H(0,2) after the above row op
        // Compute using the trick that we can form linear combinations without applying r1
        GivensRotation<T> r2(1, 2);
        if(H(1, 0) != T(0))
        {
            r2.computeConventional(H(0, 0) * H(0, 1) + H(1, 0) * H(1, 1),
                                   H(0, 0) * H(0, 2) + H(1, 0) * H(1, 2));
        }
        else
        {
            r2.computeConventional(H(0, 1), H(0, 2));
        }

        // Apply r1: A <- G^T A; accumulate into U as columnRotation (U <- U G)
        r1.rowRotation(H);

        // Apply r2 on columns; accumulate into V: V <- V G
        r2.columnRotation(H);
        r2.columnRotation(V);

        // r3: act on rows (1,2) to zero H(2,1)
        GivensRotation<T> r3(H(1, 1), H(2, 1), 1, 2);
        r3.rowRotation(H);

        // Accumulate r1 and r3 to U as column rotations: U <- U G
        r1.columnRotation(U);
        r3.columnRotation(U);
    }

    /**
 * Upper bidiagonalization for 3x3
 * After:
 *   [ * * 0
 *     0 * *
 *     0 0 * ]
 */
    template <typename MatH, typename MatU, typename MatV>
    constexpr void upper_bidiagonalize(Eigen::MatrixBase<MatH>& H,
                                       Eigen::MatrixBase<MatU>& U,
                                       Eigen::MatrixBase<MatV>& V)
    {
        U.derived().setIdentity();
        V.derived().setIdentity();
        //printf("U:  %f, %f, %f\n", U(0, 1), U(2, 1), U(1, 2));
        // First, zero H(2,0) by rotating rows (1,2)
        using T = typename MatH::Scalar;
        GivensRotation<T> r{H(1, 0), H(2, 0), 1, 2};
        r.rowRotation(H);
        r.columnRotation(U);  // accumulate U <- U G

        // Then zero chase to upper-bidiagonal
        zero_chasing(H, U, V);
    }

}  // namespace math
}  // namespace uipc::backend::cuda