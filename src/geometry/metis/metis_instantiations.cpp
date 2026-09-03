/* Derived from METIS 5.2.1 and GKlib and adapted for libuipc's C++ build.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 *
 * C++ instantiations of the METIS/GKlib helper routines. The actual
 * implementations live as type-safe
 * templates in detail/blas_templates.h; this file only provides the thin
 * libmetis__-prefixed wrappers required by the internal rename macros in
 * metis.h.
 */

#include <metis.h>
#include <algorithm>
#include <new>
#include "detail/blas_templates.h"

namespace
{

/*************************************************************************/
/*! Thin wrappers around the BLAS templates. */
/*************************************************************************/
#define METIS_WRAP_BLAS(PRFX, TYPE, OUTTYPE, KV_TYPE, SORT_FN)                                     \
    TYPE* libmetis__##PRFX##incset(size_t n, TYPE baseval, TYPE* x)                                \
    {                                                                                              \
        return metis::detail::blas_incset(n, baseval, x);                                          \
    }                                                                                              \
    TYPE libmetis__##PRFX##max(size_t n, TYPE* x, size_t incx)                                     \
    {                                                                                              \
        return metis::detail::blas_max(n, x, incx);                                                \
    }                                                                                              \
    TYPE libmetis__##PRFX##min(size_t n, TYPE* x, size_t incx)                                     \
    {                                                                                              \
        return metis::detail::blas_min(n, x, incx);                                                \
    }                                                                                              \
    size_t libmetis__##PRFX##argmax(size_t n, TYPE* x, size_t incx)                                \
    {                                                                                              \
        return metis::detail::blas_argmax(n, x, incx);                                             \
    }                                                                                              \
    size_t libmetis__##PRFX##argmin(size_t n, TYPE* x, size_t incx)                                \
    {                                                                                              \
        return metis::detail::blas_argmin(n, x, incx);                                             \
    }                                                                                              \
    size_t libmetis__##PRFX##argmax_n(size_t n, TYPE* x, size_t incx, size_t k)                    \
    {                                                                                              \
        return metis::detail::blas_argmax_n<TYPE, KV_TYPE>(n, x, incx, k, SORT_FN);                \
    }                                                                                              \
    OUTTYPE libmetis__##PRFX##sum(size_t n, TYPE* x, size_t incx)                                  \
    {                                                                                              \
        return metis::detail::blas_sum<TYPE, OUTTYPE>(n, x, incx);                                 \
    }                                                                                              \
    TYPE* libmetis__##PRFX##scale(size_t n, TYPE alpha, TYPE* x, size_t incx)                      \
    {                                                                                              \
        return metis::detail::blas_scale(n, alpha, x, incx);                                       \
    }                                                                                              \
    OUTTYPE libmetis__##PRFX##norm2(size_t n, TYPE* x, size_t incx)                                \
    {                                                                                              \
        return metis::detail::blas_norm2<TYPE, OUTTYPE>(n, x, incx);                               \
    }                                                                                              \
    OUTTYPE libmetis__##PRFX##dot(size_t n, TYPE* x, size_t incx, TYPE* y, size_t incy)            \
    {                                                                                              \
        return metis::detail::blas_dot<TYPE, OUTTYPE>(n, x, incx, y, incy);                        \
    }                                                                                              \
    TYPE* libmetis__##PRFX##axpy(size_t n, TYPE alpha, TYPE* x, size_t incx, TYPE* y, size_t incy) \
    {                                                                                              \
        return metis::detail::blas_axpy(n, alpha, x, incx, y, incy);                               \
    }

/*************************************************************************/
/*! Thin wrappers around the allocation templates. */
/*************************************************************************/
#define METIS_WRAP_ALLOC(PRFX, TYPE)                                                                 \
    TYPE* libmetis__##PRFX##malloc(size_t n, const char* msg)                                        \
    {                                                                                                \
        return metis::detail::typed_malloc<TYPE>(n, msg);                                            \
    }                                                                                                \
    TYPE* libmetis__##PRFX##realloc(TYPE* ptr, size_t n, const char* msg)                            \
    {                                                                                                \
        return metis::detail::typed_realloc<TYPE>(ptr, n, msg);                                      \
    }                                                                                                \
    TYPE* libmetis__##PRFX##smalloc(size_t n, TYPE ival, const char* msg)                            \
    {                                                                                                \
        return metis::detail::typed_smalloc<TYPE>(n, ival, msg);                                     \
    }                                                                                                \
    TYPE* libmetis__##PRFX##set(size_t n, TYPE val, TYPE* x)                                         \
    {                                                                                                \
        return metis::detail::typed_set<TYPE>(n, val, x);                                            \
    }                                                                                                \
    TYPE* libmetis__##PRFX##copy(size_t n, TYPE* a, TYPE* b)                                         \
    {                                                                                                \
        return metis::detail::typed_copy<TYPE>(n, a, b);                                             \
    }                                                                                                \
    TYPE** libmetis__##PRFX##AllocMatrix(size_t ndim1, size_t ndim2, TYPE value, const char* errmsg) \
    {                                                                                                \
        return metis::detail::typed_AllocMatrix<TYPE>(ndim1, ndim2, value, errmsg);                  \
    }                                                                                                \
    void libmetis__##PRFX##FreeMatrix(TYPE*** r_matrix, size_t ndim1, size_t ndim2)                  \
    {                                                                                                \
        metis::detail::typed_FreeMatrix<TYPE>(r_matrix, ndim1, ndim2);                               \
    }                                                                                                \
    void libmetis__##PRFX##SetMatrix(TYPE** matrix, size_t ndim1, size_t ndim2, TYPE value)          \
    {                                                                                                \
        metis::detail::typed_SetMatrix<TYPE>(matrix, ndim1, ndim2, value);                           \
    }

}  // unnamed namespace

/*************************************************************************/
/*! BLAS wrappers */
/*************************************************************************/
METIS_WRAP_BLAS(i, idx_t, idx_t, ikv_t, libmetis__ikvsortd)
METIS_WRAP_BLAS(r, real_t, real_t, rkv_t, libmetis__rkvsortd)

/*************************************************************************/
/*! Allocation wrappers */
/*************************************************************************/
METIS_WRAP_ALLOC(i, idx_t)
METIS_WRAP_ALLOC(r, real_t)
METIS_WRAP_ALLOC(ikv, ikv_t)
METIS_WRAP_ALLOC(rkv, rkv_t)

/*************************************************************************/
/*! Priority queue free-function wrappers around detail::PQueue. */
/*************************************************************************/
#define METIS_WRAP_PQUEUE(PREFIX, QUEUE_T, KEY_T, VAL_T, EMPTY_KEY)            \
    QUEUE_T* PREFIX##Create(size_t maxnodes)                                   \
    {                                                                          \
        void* storage = gk_malloc(sizeof(QUEUE_T), "gk_pqCreate: queue");      \
        if(storage == nullptr)                                                 \
            return nullptr;                                                    \
        QUEUE_T* queue = ::new(storage) QUEUE_T{};                             \
        queue->init(maxnodes, "gk_PQInit: heap");                              \
        queue->setEmptyKey(EMPTY_KEY);                                         \
        return queue;                                                          \
    }                                                                          \
    void PREFIX##Init(QUEUE_T* queue, size_t maxnodes)                         \
    {                                                                          \
        queue->init(maxnodes, "gk_PQInit: heap");                              \
        queue->setEmptyKey(EMPTY_KEY);                                         \
    }                                                                          \
    void PREFIX##Reset(QUEUE_T* queue)                                         \
    {                                                                          \
        queue->reset();                                                        \
    }                                                                          \
    void PREFIX##Free(QUEUE_T* queue)                                          \
    {                                                                          \
        if(queue)                                                              \
            queue->free();                                                     \
    }                                                                          \
    void PREFIX##Destroy(QUEUE_T* queue)                                       \
    {                                                                          \
        if(queue)                                                              \
        {                                                                      \
            queue->free();                                                     \
            queue->~QUEUE_T();                                                 \
            gk_free(reinterpret_cast<void**>(&queue), LTERM);                  \
        }                                                                      \
    }                                                                          \
    size_t PREFIX##Length(QUEUE_T* queue)                                      \
    {                                                                          \
        return queue->length();                                                \
    }                                                                          \
    int PREFIX##Insert(QUEUE_T* queue, VAL_T node, KEY_T key)                  \
    {                                                                          \
        return queue->insert(node, key);                                       \
    }                                                                          \
    int PREFIX##Delete(QUEUE_T* queue, VAL_T node)                             \
    {                                                                          \
        return queue->del(node);                                               \
    }                                                                          \
    void PREFIX##Update(QUEUE_T* queue, VAL_T node, KEY_T newkey)              \
    {                                                                          \
        queue->update(node, newkey);                                           \
    }                                                                          \
    VAL_T PREFIX##GetTop(QUEUE_T* queue)                                       \
    {                                                                          \
        return queue->getTop();                                                \
    }                                                                          \
    VAL_T PREFIX##SeeTopVal(QUEUE_T* queue)                                    \
    {                                                                          \
        return queue->seeTopVal();                                             \
    }                                                                          \
    KEY_T PREFIX##SeeTopKey(QUEUE_T* queue)                                    \
    {                                                                          \
        return queue->seeTopKey();                                             \
    }                                                                          \
    KEY_T PREFIX##SeeKey(QUEUE_T* queue, VAL_T node)                           \
    {                                                                          \
        return queue->seeKey(node);                                            \
    }                                                                          \
    int PREFIX##CheckHeap(QUEUE_T* queue)                                      \
    {                                                                          \
        return queue->checkHeap();                                             \
    }

METIS_WRAP_PQUEUE(ipq, ipq_t, idx_t, idx_t, IDX_MAX)
METIS_WRAP_PQUEUE(rpq, rpq_t, real_t, idx_t, REAL_MAX)

#undef METIS_WRAP_PQUEUE

/*************************************************************************/
/*! Random number generation (kept as the original macro). */
/*************************************************************************/
GK_MKRANDOM(i, idx_t, idx_t)

/*************************************************************************/
/*! Utility routines (kept as the original macro). */
/*************************************************************************/
GK_MKARRAY2CSR(i, idx_t)

/*************************************************************************/
/*! Deterministic C++ replacements for GKlib's embedded qsort macro. */
/*************************************************************************/
namespace
{
constexpr ptrdiff_t PARTITION_SORT_CUTOFF = 8;

template <typename T, typename Less>
void partition_large_ranges(T* first, T* last, const Less& less)
{
    while(true)
    {
        T* middle = first + ((last - first) / 2);
        if(less(*middle, *first))
            std::swap(*middle, *first);
        if(less(*last, *middle))
        {
            std::swap(*middle, *last);
            if(less(*middle, *first))
                std::swap(*middle, *first);
        }

        T* left  = first + 1;
        T* right = last - 1;
        do
        {
            while(less(*left, *middle))
                ++left;
            while(less(*middle, *right))
                --right;

            if(left < right)
            {
                std::swap(*left, *right);
                if(middle == left)
                    middle = right;
                else if(middle == right)
                    middle = left;
                ++left;
                --right;
            }
            else if(left == right)
            {
                ++left;
                --right;
                break;
            }
        } while(left <= right);

        const bool sort_left  = right - first > PARTITION_SORT_CUTOFF;
        const bool sort_right = last - left > PARTITION_SORT_CUTOFF;
        if(!sort_left && !sort_right)
            return;
        if(!sort_left)
        {
            first = left;
            continue;
        }
        if(!sort_right)
        {
            last = right;
            continue;
        }

        if(right - first < last - left)
        {
            partition_large_ranges(first, right, less);
            first = left;
        }
        else
        {
            partition_large_ranges(left, last, less);
            last = right;
        }
    }
}

template <typename T, typename Less>
void deterministic_partition_sort(size_t count, T* values, const Less& less)
{
    if(count < 2)
        return;

    if(count > static_cast<size_t>(PARTITION_SORT_CUTOFF))
        partition_large_ranges(values, values + count - 1, less);

    const size_t sentinel_limit = count - 1 < static_cast<size_t>(PARTITION_SORT_CUTOFF) ?
                                      count - 1 :
                                      static_cast<size_t>(PARTITION_SORT_CUTOFF);
    size_t smallest = 0;
    for(size_t i = 1; i <= sentinel_limit; ++i)
    {
        if(less(values[i], values[smallest]))
            smallest = i;
    }
    if(smallest != 0)
        std::swap(values[0], values[smallest]);

    for(size_t current = 2; current < count; ++current)
    {
        const T value     = values[current];
        size_t  insert_at = current;
        while(insert_at > 0 && less(value, values[insert_at - 1]))
            --insert_at;
        for(size_t i = current; i > insert_at; --i)
            values[i] = values[i - 1];
        values[insert_at] = value;
    }
}
}  // namespace

void libmetis__isorti(size_t n, idx_t* base)
{
    deterministic_partition_sort(n, base, [](idx_t a, idx_t b) { return a < b; });
}

void libmetis__isortd(size_t n, idx_t* base)
{
    deterministic_partition_sort(n, base, [](idx_t a, idx_t b) { return a > b; });
}

void libmetis__rsorti(size_t n, real_t* base)
{
    deterministic_partition_sort(
        n, base, [](real_t a, real_t b) { return a < b; });
}

void libmetis__rsortd(size_t n, real_t* base)
{
    deterministic_partition_sort(
        n, base, [](real_t a, real_t b) { return a > b; });
}

void libmetis__ikvsorti(size_t n, ikv_t* base)
{
    deterministic_partition_sort(
        n, base, [](const ikv_t& a, const ikv_t& b) { return a.key < b.key; });
}

void libmetis__ikvsortii(size_t n, ikv_t* base)
{
    deterministic_partition_sort(n,
                                 base,
                                 [](const ikv_t& a, const ikv_t& b) {
                                     return a.key < b.key
                                            || (a.key == b.key && a.val < b.val);
                                 });
}

void libmetis__ikvsortd(size_t n, ikv_t* base)
{
    deterministic_partition_sort(
        n, base, [](const ikv_t& a, const ikv_t& b) { return a.key > b.key; });
}

void libmetis__rkvsortd(size_t n, rkv_t* base)
{
    deterministic_partition_sort(
        n, base, [](const rkv_t& a, const rkv_t& b) { return a.key > b.key; });
}

void libmetis__rkvsorti(size_t n, rkv_t* base)
{
    deterministic_partition_sort(
        n, base, [](const rkv_t& a, const rkv_t& b) { return a.key < b.key; });
}

void libmetis__uvwsorti(size_t n, uvw_t* base)
{
    deterministic_partition_sort(n,
                                 base,
                                 [](const uvw_t& a, const uvw_t& b) {
                                     return a.u < b.u || (a.u == b.u && a.v < b.v);
                                 });
}
