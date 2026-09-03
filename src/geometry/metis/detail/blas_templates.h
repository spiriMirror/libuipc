/* Derived from GKlib and adapted for libuipc's C++ build.
 * See ../LICENSE-GKlib.
 *
 * Type-safe C++ replacements for the GK_MKBLAS / GK_MKALLOC macro families.
 * These templates implement the same operations as the original METIS/GKlib
 * macros but are type-safe and considerably shorter.  They are intentionally
 * kept header-only so that each concrete wrapper in metis_instantiations.cpp
 * can be inlined into the same code the old macros produced.
 */

#ifndef METIS_BLAS_TEMPLATES_H
#define METIS_BLAS_TEMPLATES_H

#include <cmath>
#include <cstddef>
#include <cstring>

#include "GKlib.h"

namespace metis
{
namespace detail
{

    /*************************************************************************/
    /*! BLAS-like scalar routines */
    /*************************************************************************/

    template <typename T>
    inline T* blas_incset(size_t n, T baseval, T* x)
    {
        for(size_t i = 0; i < n; ++i)
            x[i] = baseval + static_cast<T>(i);
        return x;
    }

    template <typename T>
    inline T blas_max(size_t n, T* x, size_t incx)
    {
        if(n == 0)
            return T{};
        T mx = *x;
        for(size_t i = 1; i < n; ++i)
        {
            x += incx;
            if(*x > mx)
                mx = *x;
        }
        return mx;
    }

    template <typename T>
    inline T blas_min(size_t n, T* x, size_t incx)
    {
        if(n == 0)
            return T{};
        T mn = *x;
        for(size_t i = 1; i < n; ++i)
        {
            x += incx;
            if(*x < mn)
                mn = *x;
        }
        return mn;
    }

    template <typename T>
    inline size_t blas_argmax(size_t n, T* x, size_t incx)
    {
        size_t best = 0;
        for(size_t i = 1; i < n; ++i)
        {
            if(x[i * incx] > x[best * incx])
                best = i;
        }
        return best;
    }

    template <typename T>
    inline size_t blas_argmin(size_t n, T* x, size_t incx)
    {
        size_t best = 0;
        for(size_t i = 1; i < n; ++i)
        {
            if(x[i * incx] < x[best * incx])
                best = i;
        }
        return best;
    }

    template <typename T, typename KV, typename SortFn>
    inline size_t blas_argmax_n(size_t n, T* x, size_t incx, size_t k, SortFn sort_fn)
    {
        KV* cand = static_cast<KV*>(gk_malloc(sizeof(KV) * n, "GK_ARGMAX_N: cand"));

        for(size_t i = 0, j = 0; i < n; ++i, j += incx)
        {
            cand[i].val = static_cast<decltype(KV::val)>(i);
            cand[i].key = x[j];
        }
        sort_fn(n, cand);

        size_t max_n = cand[k - 1].val;
        gk_free((void**)&cand, LTERM);
        return max_n;
    }

    template <typename T, typename OutT>
    inline OutT blas_sum(size_t n, T* x, size_t incx)
    {
        OutT s = 0;
        for(size_t i = 0; i < n; ++i, x += incx)
            s += static_cast<OutT>(*x);
        return s;
    }

    template <typename T>
    inline T* blas_scale(size_t n, T alpha, T* x, size_t incx)
    {
        for(size_t i = 0; i < n; ++i, x += incx)
            *x *= alpha;
        // Match GK_MKBLAS exactly: unlike axpy(), scale() returns the advanced
        // iterator after the final stride.
        return x;
    }

    template <typename T, typename OutT>
    inline OutT blas_norm2(size_t n, T* x, size_t incx)
    {
        OutT partial = 0;
        for(size_t i = 0; i < n; ++i, x += incx)
            partial += static_cast<OutT>((*x) * (*x));
        return partial > 0 ?
                   static_cast<OutT>(std::sqrt(static_cast<double>(partial))) :
                   OutT{};
    }

    template <typename T, typename OutT>
    inline OutT blas_dot(size_t n, T* x, size_t incx, T* y, size_t incy)
    {
        OutT partial = 0;
        for(size_t i = 0; i < n; ++i, x += incx, y += incy)
            partial += static_cast<OutT>((*x) * (*y));
        return partial;
    }

    template <typename T>
    inline T* blas_axpy(size_t n, T alpha, T* x, size_t incx, T* y, size_t incy)
    {
        T* y_in = y;
        for(size_t i = 0; i < n; ++i, x += incx, y += incy)
            *y += alpha * (*x);
        return y_in;
    }

    /*************************************************************************/
    /*! Typed allocation / set / copy routines */
    /*************************************************************************/

    template <typename T>
    inline T* typed_malloc(size_t n, const char* msg)
    {
        return static_cast<T*>(gk_malloc(sizeof(T) * n, msg));
    }

    template <typename T>
    inline T* typed_realloc(T* ptr, size_t n, const char* msg)
    {
        return static_cast<T*>(gk_realloc(static_cast<void*>(ptr), sizeof(T) * n, msg));
    }

    template <typename T>
    inline T* typed_set(size_t n, T val, T* x)
    {
        for(size_t i = 0; i < n; ++i)
            x[i] = val;
        return x;
    }

    template <typename T>
    inline T* typed_smalloc(size_t n, T ival, const char* msg)
    {
        T* ptr = typed_malloc<T>(n, msg);
        if(ptr == nullptr)
            return nullptr;
        return typed_set<T>(n, ival, ptr);
    }

    template <typename T>
    inline T* typed_copy(size_t n, T* a, T* b)
    {
        return static_cast<T*>(
            memmove(static_cast<void*>(b), static_cast<void*>(a), sizeof(T) * n));
    }

    template <typename T>
    inline T** typed_AllocMatrix(size_t ndim1, size_t ndim2, T value, const char* errmsg)
    {
        T** matrix = static_cast<T**>(gk_malloc(ndim1 * sizeof(T*), errmsg));
        if(matrix == nullptr)
            return nullptr;

        for(size_t i = 0; i < ndim1; ++i)
        {
            matrix[i] = typed_smalloc<T>(ndim2, value, errmsg);
            if(matrix[i] == nullptr)
            {
                for(size_t j = 0; j < i; ++j)
                    gk_free((void**)&matrix[j], LTERM);
                gk_free((void**)&matrix, LTERM);
                return nullptr;
            }
        }
        return matrix;
    }

    template <typename T>
    inline void typed_FreeMatrix(T*** r_matrix, size_t ndim1, size_t /*ndim2*/)
    {
        if(r_matrix == nullptr || *r_matrix == nullptr)
            return;

        T** matrix = *r_matrix;
        for(size_t i = 0; i < ndim1; ++i)
            gk_free((void**)&matrix[i], LTERM);

        gk_free((void**)r_matrix, LTERM);
    }

    template <typename T>
    inline void typed_SetMatrix(T** matrix, size_t ndim1, size_t ndim2, T value)
    {
        for(size_t i = 0; i < ndim1; ++i)
            for(size_t j = 0; j < ndim2; ++j)
                matrix[i][j] = value;
    }

}  // namespace detail
}  // namespace metis

#endif  // METIS_BLAS_TEMPLATES_H
