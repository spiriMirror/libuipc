#pragma once
#include <algorithm/givens.hpp>
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace math
{
    template <typename T>
    UIPC_GENERIC constexpr void swap(T& a, T& b)
    {
        auto s = a;
        a      = b;
        b      = s;
    }

    template <typename T>
    UIPC_GENERIC constexpr T abd(T& a)
    {
        return a > 0 ? a : -a;
    }

    template <typename T>
    UIPC_GENERIC constexpr void polar_decomposition(const Eigen::Matrix<T, 2, 2>& A,
                                       Eigen::Matrix<T, 2, 2>&       S,
                                       GivensRotation<T>&            R) noexcept
    {
        S = A;
        Eigen::Matrix<T, 2, 1> x{A(0, 0) + A(1, 1), A(1, 0) - A(0, 1)};
        auto                   d = x.norm();
        if(d != 0)
        {
            R.c = x(0) / d;
            R.s = -x(1) / d;
        }
        else
        {
            R.c = 1;
            R.s = 0;
        }
        R.rowRotation(S);
    }

    template <typename T>
    UIPC_GENERIC constexpr void qr_svd2x2(const Eigen::Matrix<T, 2, 2>& A,
                             Eigen::Matrix<T, 2, 1>&       S,
                             GivensRotation<T>&            U,
                             GivensRotation<T>&            V) noexcept
    {
        Eigen::Matrix<T, 2, 2> S_sym;
        polar_decomposition(A, S_sym, U);

        T    cosine{}, sine{};
        auto x{S_sym(0, 0)}, y{S_sym(0, 1)}, z{S_sym(1, 1)};
        auto y2 = y * y;

        if(y2 == 0)
        {  // S is already diagonal
            cosine = 1;
            sine   = 0;
            S(0)   = x;
            S(1)   = z;
        }
        else
        {
            auto tau = (T)0.5 * (x - z);
            T    w{std::sqrt(tau * tau + y2)}, t{};
            if(tau > 0)  // tau + w > w > y > 0 ==> division is safe
                t = y / (tau + w);
            else  // tau - w < -w < -y < 0 ==> division is safe
                t = y / (tau - w);
            cosine = 1 / std::sqrt(t * t + (T)1);
            sine   = -t * cosine;

            T c2    = cosine * cosine;
            T _2csy = 2 * cosine * sine * y;
            T s2    = sine * sine;

            S(0) = c2 * x - _2csy + s2 * z;
            S(1) = s2 * x + _2csy + c2 * z;
        }

        // Sorting
        if(S(0) < S(1))
        {
            swap(S(0), S(1));
            V.c = -sine;
            V.s = cosine;
        }
        else
        {
            V.c = cosine;
            V.s = sine;
        }
        U *= V;
    }

    template <typename T>
    UIPC_GENERIC constexpr T wilkinson_shift(const T a1, const T b1, const T a2) noexcept
    {
        T d  = (T)0.5 * (a1 - a2);
        T bs = b1 * b1;
        T mu = a2 - std::copysign(bs / (std::abs(d) + std::sqrt(d * d + bs)), d);
        return mu;
    }

    template <typename T>
    UIPC_GENERIC constexpr void flip_sign(int j, Eigen::Matrix<T, 3, 3>& U, Eigen::Matrix<T, 3, 1>& S) noexcept
    {
        S(j)     = -S(j);
        U.col(j) = -U.col(j);
    }

    template <int t, typename T>
    UIPC_GENERIC constexpr void sort_sigma(Eigen::Matrix<T, 3, 3>& U,
                              Eigen::Matrix<T, 3, 1>& sigma,
                              Eigen::Matrix<T, 3, 3>& V) noexcept
    {
        /// t == 0
        if constexpr(t == 0)
        {
            if(std::abs(sigma(1)) >= std::abs(sigma(2)))
            {
                if(sigma(1) < 0)
                {
                    flip_sign(1, U, sigma);
                    flip_sign(2, U, sigma);
                }
                return;
            }

            if(sigma(2) < 0)
            {
                flip_sign(1, U, sigma);
                flip_sign(2, U, sigma);
            }

            swap(sigma(1), sigma(2));
            U.col(1).swap(U.col(2));
            V.col(1).swap(V.col(2));

            if(sigma(1) > sigma(0))
            {
                swap(sigma(0), sigma(1));
                U.col(0).swap(U.col(1));
                V.col(0).swap(V.col(1));
            }
            else
            {
                U.col(2) = -U.col(2);
                V.col(2) = -V.col(2);
            }
        }
        /// t == 1
        else if constexpr(t == 1)
        {
            if(std::abs(sigma(0)) >= sigma(1))
            {
                if(sigma(0) < 0)
                {
                    flip_sign(0, U, sigma);
                    flip_sign(2, U, sigma);
                }
                return;
            }

            swap(sigma(0), sigma(1));
            U.col(0).swap(U.col(1));
            V.col(0).swap(V.col(1));

            if(std::abs(sigma(1)) < std::abs(sigma(2)))
            {
                swap(sigma(1), sigma(2));
                U.col(1).swap(U.col(2));
                V.col(1).swap(V.col(2));
            }
            else
            {
                U.col(1) = -U.col(1);
                V.col(1) = -V.col(1);
            }

            if(sigma(1) < 0)
            {
                flip_sign(1, U, sigma);
                flip_sign(2, U, sigma);
            }
        }
    }

    template <int t, typename T>
    UIPC_GENERIC constexpr void process(Eigen::Matrix<T, 3, 3>& B,
                           Eigen::Matrix<T, 3, 3>& U,
                           Eigen::Matrix<T, 3, 1>& S,
                           Eigen::Matrix<T, 3, 3>& V) noexcept
    {
        GivensRotation<T> u{0, 1};
        GivensRotation<T> v{0, 1};
        constexpr int     other   = (t == 1) ? 0 : 2;
        S(other)                  = B(other, other);
        Eigen::Matrix<T, 2, 2> B_ = B.template block<2, 2>(t, t);
        Eigen::Matrix<T, 2, 1> S_;
        qr_svd2x2(B_, S_, u, v);
        S(t)                         = S_(0);
        S(t + 1)                     = S_(1);

        u.rowi += t;
        u.rowk += t;
        v.rowi += t;
        v.rowk += t;
        u.columnRotation(U);
        v.columnRotation(V);
    }


    template <typename T>
    UIPC_GENERIC constexpr void qr_svd(const Eigen::Matrix<T, 3, 3>& A,
                          Eigen::Matrix<T, 3, 1>&       S,
                          Eigen::Matrix<T, 3, 3>&       U,
                          Eigen::Matrix<T, 3, 3>&       V) noexcept
    {
        U.setIdentity();
        V.setIdentity();

        Eigen::Matrix<T, 3, 3> B = A;

        upper_bidiagonalize(B, U, V);

        GivensRotation<T> r{0, 1};

        T alpha[3] = {B(0, 0), B(1, 1), B(2, 2)};
        T beta[2]  = {B(0, 1), B(1, 2)};
        T gamma[2] = {alpha[0] * beta[0], alpha[1] * beta[1]};

        constexpr auto eta = std::numeric_limits<T>::epsilon() * (T)128;
        T              tol = eta
                * std::max((T)0.5
                               * std::sqrt(alpha[0] * alpha[0] + alpha[1] * alpha[1]
                                           + alpha[2] * alpha[2]
                                           + beta[0] * beta[0] + beta[1] * beta[1]),
                           (T)1);

        while(abd(alpha[0]) > tol && abs(alpha[1]) > tol && abs(alpha[2]) > tol
              && abs(beta[0]) > tol && abs(beta[1]) > tol)
        {
            auto mu = wilkinson_shift(alpha[1] * alpha[1] + beta[0] * beta[0],
                                      gamma[1],
                                      alpha[2] * alpha[2] + beta[1] * beta[1]);

            r.computeConventional(alpha[0] * alpha[0] - mu, gamma[0]);
            r.columnRotation(B);
            r.columnRotation(V);
            zero_chasing(B, U, V);

            alpha[0] = B(0, 0);
            alpha[1] = B(1, 1);
            alpha[2] = B(2, 2);
            beta[0]  = B(0, 1);
            beta[1]  = B(1, 2);
            gamma[0] = alpha[0] * beta[0];
            gamma[1] = alpha[1] * beta[1];
        }

        if(std::abs(beta[1]) <= tol)
        {
            process<0>(B, U, S, V);
            sort_sigma<0>(U, S, V);
        }
        else if(std::abs(beta[0]) <= tol)
        {
            process<1>(B, U, S, V);
            sort_sigma<1>(U, S, V);
        }
        else if(std::abs(alpha[1]) <= tol)
        {
            GivensRotation<T> r1(1, 2);
            r1.computeUnconventional(B(1, 2), B(2, 2));
            r1.rowRotation(B);
            r1.columnRotation(U);
            process<0>(B, U, S, V);
            sort_sigma<0>(U, S, V);
        }
        else if(std::abs(alpha[2]) <= tol)
        {
            GivensRotation<T> r1(1, 2);
            r1.computeConventional(B(1, 1), B(1, 2));
            r1.columnRotation(B);
            r1.columnRotation(V);

            GivensRotation<T> r2(0, 2);
            r2.computeConventional(B(0, 0), B(0, 2));
            r2.columnRotation(B);
            r2.columnRotation(V);

            process<0>(B, U, S, V);
            sort_sigma<0>(U, S, V);
        }
        else if(std::abs(alpha[0]) <= tol)
        {
            GivensRotation<T> r1(0, 1);
            r1.computeUnconventional(B(0, 1), B(1, 1));
            r1.rowRotation(B);
            r1.columnRotation(U);

            GivensRotation<T> r2(0, 2);
            r2.computeUnconventional(B(0, 2), B(2, 2));
            r2.rowRotation(B);
            r2.columnRotation(U);

            process<1>(B, U, S, V);
            sort_sigma<1>(U, S, V);
        }
    }
}  // namespace math

}  // namespace uipc::backend::cuda