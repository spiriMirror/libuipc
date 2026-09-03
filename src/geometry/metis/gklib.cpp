/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 *
 * This minimal merged file removes unused source files and
 * unused GK_MK* macro instantiations while preserving the algorithm
 * used by METIS_PartGraphKway.
 */

#include "GKlib.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#include <type_traits>

/************************ blas.c ************************/
/*!
\file blas.c
\brief This file contains GKlib's implementation of BLAS-like routines

The BLAS routines that are currently implemented are mostly level-one.
They follow a naming convention of the type gk_[type][name], where
[type] is one of c, i, f, and d, based on C's four standard scalar
datatypes of characters, integers, floats, and doubles.

These routines are implemented using a generic macro template,
which is used for code generation.

\date   Started 9/28/95
\author George
\version\verbatim $Id: blas.c 14330 2013-05-18 12:15:15Z karypis $ \endverbatim
*/


/*************************************************************************/
/*! Use the templates to generate BLAS routines for the scalar data types */
/*************************************************************************/


/************************ error.c ************************/
/*!
\file  error.c
\brief Various error-handling functions

This file contains functions dealing with error reporting and termination

\author George
\date 1/1/2007
\version\verbatim $Id: error.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#define _GK_ERROR_C_ /* this is needed to properly declare the gk_jub* variables
                         as an extern function in GKlib.h */


/* These are the jmp_buf for the graceful exit in case of severe errors.
   Multiple buffers are defined to allow for recursive invokation. */
#define MAX_JBUFS 128
__thread int     gk_cur_jbufs = -1;
__thread jmp_buf gk_jbufs[MAX_JBUFS];
__thread jmp_buf gk_jbuf;

typedef void (*gksighandler_t)(int);

/* These are the holders of the old singal handlers for the trapped signals */
static __thread gksighandler_t old_SIGMEM_handler; /* Custom signal */
static __thread gksighandler_t old_SIGERR_handler; /* Custom signal */
// signal() installs process-wide handlers, whereas the jump buffers above are
// thread-local.  Official GKlib saves/restores the handlers per thread, which
// races when independent METIS calls run concurrently.  Keep one shared
// installation alive until the last active trap exits; nested jump targets
// remain thread-local and retain the original error semantics.
static std::mutex     signal_handler_mutex;
static size_t         signal_trap_users         = 0;
static gksighandler_t shared_old_SIGMEM_handler = SIG_DFL;
static gksighandler_t shared_old_SIGERR_handler = SIG_DFL;

/* The following is used to control if the gk_errexit() will actually abort or not.
   There is always a single copy of this variable */
static int gk_exit_on_error = 1;


/*************************************************************************/
/*! This function sets the gk_exit_on_error variable
 */
/*************************************************************************/
void gk_set_exit_on_error(int value)
{
    gk_exit_on_error = value;
}


/*************************************************************************/
/*! This function prints an error message and exits
 */
/*************************************************************************/
void errexit(const char* f_str, ...)
{
    va_list argp;

    va_start(argp, f_str);
    vfprintf(stderr, f_str, argp);
    va_end(argp);

    if(strlen(f_str) == 0 || f_str[strlen(f_str) - 1] != '\n')
        fprintf(stderr, "\n");
    fflush(stderr);

    if(gk_exit_on_error)
        exit(-2);

    /* abort(); */
}


/*************************************************************************/
/*! This function prints an error message and raises a signum signal
 */
/*************************************************************************/
void gk_errexit(int signum, const char* f_str, ...)
{
    va_list argp;

    va_start(argp, f_str);
    vfprintf(stderr, f_str, argp);
    va_end(argp);

    fprintf(stderr, "\n");
    fflush(stderr);

    if(gk_exit_on_error)
        raise(signum);
}


/***************************************************************************/
/*! This function sets a number of signal handlers and sets the return point
    of a longjmp
*/
/***************************************************************************/
int gk_sigtrap()
{
    if(gk_cur_jbufs + 1 >= MAX_JBUFS)
        return 0;

    gk_cur_jbufs++;

    {
        std::lock_guard<std::mutex> lock(signal_handler_mutex);
        if(signal_trap_users++ == 0)
        {
            shared_old_SIGMEM_handler = signal(SIGMEM, gk_sigthrow);
            shared_old_SIGERR_handler = signal(SIGERR, gk_sigthrow);
        }
    }

    return 1;
}


/***************************************************************************/
/*! This function sets the handlers for the signals to their default handlers
 */
/***************************************************************************/
int gk_siguntrap()
{
    if(gk_cur_jbufs == -1)
        return 0;

    int removed_trap = 0;
    {
        std::lock_guard<std::mutex> lock(signal_handler_mutex);
        if(signal_trap_users > 0)
        {
            removed_trap = 1;
            if(--signal_trap_users == 0)
            {
                signal(SIGMEM, shared_old_SIGMEM_handler);
                signal(SIGERR, shared_old_SIGERR_handler);
            }
        }
    }

    /* Keep the thread-local nesting depth balanced even if a mismatched caller
     exposed an already-zero process-wide count. */
    gk_cur_jbufs--;

    return removed_trap;
}


/*************************************************************************/
/*! This function is the custome signal handler, which all it does is to
    perform a longjump to the most recent saved environment
 */
/*************************************************************************/
void gk_sigthrow(int signum)
{
    longjmp(gk_jbufs[gk_cur_jbufs], signum);
}


/***************************************************************************
* This function sets a number of signal handlers and sets the return point
* of a longjmp
****************************************************************************/
void gk_SetSignalHandlers()
{
    old_SIGMEM_handler = signal(SIGMEM, gk_NonLocalExit_Handler);
    old_SIGERR_handler = signal(SIGERR, gk_NonLocalExit_Handler);
}


/***************************************************************************
* This function sets the handlers for the signals to their default handlers
****************************************************************************/
void gk_UnsetSignalHandlers()
{
    signal(SIGMEM, old_SIGMEM_handler);
    signal(SIGERR, old_SIGERR_handler);
}


/*************************************************************************
* This function is the handler for SIGUSR1 that implements the cleaning up
* process prior to a non-local exit.
**************************************************************************/
void gk_NonLocalExit_Handler(int signum)
{
    longjmp(gk_jbuf, signum);
}


/*************************************************************************/
/*! \brief Thread-safe implementation of strerror() */
/**************************************************************************/
char* gk_strerror(int errnum)
{
#if defined(WIN32) || defined(__MINGW32__)
    return strerror(errnum);
#else
#ifndef SUNOS
    static __thread char buf[1024];

    const auto normalize_result = [errnum](auto result, char* buffer, size_t size)
    {
        using Result = decltype(result);
        if constexpr(std::is_pointer_v<Result>)
        {
            if(result == nullptr)
                std::snprintf(buffer, size, "Unknown error %d", errnum);
            else if(result != buffer)
                std::snprintf(buffer, size, "%s", result);
        }
        else if(result != 0)
        {
            std::snprintf(buffer, size, "Unknown error %d", errnum);
        }
    };

    normalize_result(strerror_r(errnum, buf, sizeof(buf)), buf, sizeof(buf));

    buf[sizeof(buf) - 1] = '\0';
    return buf;
#else
    return strerror(errnum);
#endif
#endif
}


/*************************************************************************
* This function prints a backtrace of calling functions
**************************************************************************/
void PrintBackTrace()
{
#ifdef HAVE_EXECINFO_H
    void*  array[10];
    int    i, size;
    char** strings;

    size    = backtrace(array, 10);
    strings = backtrace_symbols(array, size);

    printf("Obtained %d stack frames.\n", size);
    for(i = 0; i < size; i++)
    {
        printf("%s\n", strings[i]);
    }
    free(strings);
#endif
}
/************************ gk_util.c ************************/
/*!
\file  util.c
\brief Various utility routines

\date   Started 4/12/2007
\author George
\version\verbatim $Id: gk_util.c 16223 2014-02-15 21:34:09Z karypis $ \endverbatim
*/


/*************************************************************************
* This file randomly permutes the contents of an array.
* flag == 0, don't initialize perm
* flag == 1, set p[i] = i
**************************************************************************/
void gk_RandomPermute(size_t n, int* p, int flag)
{
    size_t i, u, v;
    int    tmp;

    if(flag == 1)
    {
        for(i = 0; i < n; i++)
            p[i] = i;
    }

    for(i = 0; i < n / 2; i++)
    {
        v = RandomInRange(n);
        u = RandomInRange(n);
        gk_SWAP(p[v], p[u], tmp);
    }
}


/************************************************************************/
/*!
\brief Converts an element-based set membership into a CSR-format set-based
       membership.

For example, it takes an array such as part[] that stores where each
element belongs to and returns a pair of arrays (pptr[], pind[]) that
store in CSF format the list of elements belonging in each partition.

\param n
  the number of elements in the array (e.g., # of vertices)
\param range
  the cardinality of the set (e.g., # of partitions)
\param array
  the array that stores the per-element set membership
\param ptr
  the array that will store the starting indices in ind for
  the elements of each set. This is filled by the routine and
  its size should be at least range+1.
\param ind
  the array that stores consecutively which elements belong to
  each set. The size of this array should be n.
*/
/************************************************************************/
void gk_array2csr(size_t n, size_t range, int* array, int* ptr, int* ind)
{
    size_t i;

    gk_iset(range + 1, 0, ptr);

    for(i = 0; i < n; i++)
        ptr[array[i]]++;

    /* Compute the ptr, ind structure */
    MAKECSR(i, range, ptr);
    for(i = 0; i < n; i++)
        ind[ptr[array[i]]++] = i;
    SHIFTCSR(i, range, ptr);
}


/*************************************************************************
* This function returns the log2(x)
**************************************************************************/
int gk_log2(int a)
{
    size_t i;

    for(i = 1; a > 1; i++, a = a >> 1)
        ;
    return i - 1;
}


/*************************************************************************
* This function checks if the argument is a power of 2
**************************************************************************/
int gk_ispow2(int a)
{
    return (a == (1 << gk_log2(a)));
}


/*************************************************************************
* This function returns the log2(x)
**************************************************************************/
float gk_flog2(float a)
{
    return log(a) / log(2.0);
}


/************************ mcore.c ************************/
/*!
\file
\brief Functions dealing with creating and allocating mcores

\date Started 5/30/11
\author George
\author Copyright 1997-2011, Regents of the University of Minnesota
\version $Id: mcore.c 13953 2013-03-30 16:20:07Z karypis $
*/


/*************************************************************************/
/*! This function creates an mcore
 */
/*************************************************************************/
gk_mcore_t* gk_mcoreCreate(size_t coresize)
{
    gk_mcore_t* mcore;

    mcore = (gk_mcore_t*)gk_malloc(sizeof(gk_mcore_t), "gk_mcoreCreate: mcore");
    memset(mcore, 0, sizeof(gk_mcore_t));

    mcore->coresize = coresize;
    mcore->corecpos = 0;

    mcore->core =
        (coresize == 0 ? NULL : gk_malloc(mcore->coresize, "gk_mcoreCreate: core"));

    /* allocate the memory for keeping track of malloc ops */
    mcore->nmops = 2048;
    mcore->cmop  = 0;
    mcore->mops  = (gk_mop_t*)gk_malloc(mcore->nmops * sizeof(gk_mop_t),
                                       "gk_mcoreCreate: mcore->mops");

    return mcore;
}


/*************************************************************************/
/*! This function creates an mcore. This version is used for gkmcore.
 */
/*************************************************************************/
gk_mcore_t* gk_gkmcoreCreate()
{
    gk_mcore_t* mcore;

    if((mcore = (gk_mcore_t*)malloc(sizeof(gk_mcore_t))) == NULL)
        return NULL;
    memset(mcore, 0, sizeof(gk_mcore_t));

    /* allocate the memory for keeping track of malloc ops */
    mcore->nmops = 2048;
    mcore->cmop  = 0;
    if((mcore->mops = (gk_mop_t*)malloc(mcore->nmops * sizeof(gk_mop_t))) == NULL)
    {
        free(mcore);
        return NULL;
    }

    return mcore;
}


/*************************************************************************/
/*! This function destroys an mcore.
 */
/*************************************************************************/
void gk_mcoreDestroy(gk_mcore_t** r_mcore, int showstats)
{
    gk_mcore_t* mcore = *r_mcore;

    if(mcore == NULL)
        return;

    if(showstats)
        printf(
            "\n gk_mcore statistics\n"
            "           coresize: %12zu         nmops: %12zu  cmop: %6zu\n"
            "        num_callocs: %12zu   num_hallocs: %12zu\n"
            "       size_callocs: %12zu  size_hallocs: %12zu\n"
            "        cur_callocs: %12zu   cur_hallocs: %12zu\n"
            "        max_callocs: %12zu   max_hallocs: %12zu\n",
            mcore->coresize,
            mcore->nmops,
            mcore->cmop,
            mcore->num_callocs,
            mcore->num_hallocs,
            mcore->size_callocs,
            mcore->size_hallocs,
            mcore->cur_callocs,
            mcore->cur_hallocs,
            mcore->max_callocs,
            mcore->max_hallocs);

    if(mcore->cur_callocs != 0 || mcore->cur_hallocs != 0 || mcore->cmop != 0)
    {
        printf(
            "***Warning: mcore memory was not fully freed when destroyed.\n"
            " cur_callocs: %6zu  cur_hallocs: %6zu cmop: %6zu\n",
            mcore->cur_callocs,
            mcore->cur_hallocs,
            mcore->cmop);
    }

    gk_free((void**)&mcore->core, &mcore->mops, &mcore, LTERM);

    *r_mcore = NULL;
}


/*************************************************************************/
/*! This function destroys an mcore. This version is for gkmcore.
 */
/*************************************************************************/
void gk_gkmcoreDestroy(gk_mcore_t** r_mcore, int showstats)
{
    gk_mcore_t* mcore = *r_mcore;

    if(mcore == NULL)
        return;

    if(showstats)
        printf(
            "\n gk_mcore statistics\n"
            "         nmops: %12zu  cmop: %6zu\n"
            "   num_hallocs: %12zu\n"
            "  size_hallocs: %12zu\n"
            "   cur_hallocs: %12zu\n"
            "   max_hallocs: %12zu\n",
            mcore->nmops,
            mcore->cmop,
            mcore->num_hallocs,
            mcore->size_hallocs,
            mcore->cur_hallocs,
            mcore->max_hallocs);

    if(mcore->cur_hallocs != 0 || mcore->cmop != 0)
    {
        printf(
            "***Warning: mcore memory was not fully freed when destroyed.\n"
            " cur_hallocs: %6zu cmop: %6zu\n",
            mcore->cur_hallocs,
            mcore->cmop);
    }

    free(mcore->mops);
    free(mcore);

    *r_mcore = NULL;
}


/*************************************************************************/
/*! This function allocate space from the core/heap
 */
/*************************************************************************/
void* gk_mcoreMalloc(gk_mcore_t* mcore, size_t nbytes)
{
    void* ptr;

    /* pad to make pointers 8-byte aligned */
    nbytes += (nbytes % 8 == 0 ? 0 : 8 - nbytes % 8);

    if(mcore->corecpos + nbytes < mcore->coresize)
    {
        /* service this request from the core */
        ptr = ((char*)mcore->core) + mcore->corecpos;
        mcore->corecpos += nbytes;

        gk_mcoreAdd(mcore, GK_MOPT_CORE, nbytes, ptr);
    }
    else
    {
        /* service this request from the heap */
        ptr = gk_malloc(nbytes, "gk_mcoremalloc: ptr");

        gk_mcoreAdd(mcore, GK_MOPT_HEAP, nbytes, ptr);
    }

    /*
  printf("MCMALLOC: %zu %d %8zu\n", mcore->cmop-1,
      mcore->mops[mcore->cmop-1].type, mcore->mops[mcore->cmop-1].nbytes);
  */

    return ptr;
}


/*************************************************************************/
/*! This function sets a marker in the stack of malloc ops to be used
    subsequently for freeing purposes
 */
/*************************************************************************/
void gk_mcorePush(gk_mcore_t* mcore)
{
    gk_mcoreAdd(mcore, GK_MOPT_MARK, 0, NULL);
    /* printf("MCPPUSH:   %zu\n", mcore->cmop-1); */
}


/*************************************************************************/
/*! This function sets a marker in the stack of malloc ops to be used
    subsequently for freeing purposes. This is the gkmcore version.
 */
/*************************************************************************/
void gk_gkmcorePush(gk_mcore_t* mcore)
{
    gk_gkmcoreAdd(mcore, GK_MOPT_MARK, 0, NULL);
    /* printf("MCPPUSH:   %zu\n", mcore->cmop-1); */
}


/*************************************************************************/
/*! This function frees all mops since the last push
 */
/*************************************************************************/
void gk_mcorePop(gk_mcore_t* mcore)
{
    while(mcore->cmop > 0)
    {
        mcore->cmop--;
        switch(mcore->mops[mcore->cmop].type)
        {
            case GK_MOPT_MARK: /* push marker */
                goto DONE;
                break;

            case GK_MOPT_CORE: /* core free */
                if(mcore->corecpos < mcore->mops[mcore->cmop].nbytes)
                    errexit("Internal Error: wspace's core is about to be over-freed [%zu, %zu, %zd]\n",
                            mcore->coresize,
                            mcore->corecpos,
                            mcore->mops[mcore->cmop].nbytes);

                mcore->corecpos -= mcore->mops[mcore->cmop].nbytes;
                mcore->cur_callocs -= mcore->mops[mcore->cmop].nbytes;
                break;

            case GK_MOPT_HEAP: /* heap free */
                gk_free((void**)&mcore->mops[mcore->cmop].ptr, LTERM);
                mcore->cur_hallocs -= mcore->mops[mcore->cmop].nbytes;
                break;

            default:
                gk_errexit(SIGMEM,
                           "Unknown mop type of %d\n",
                           mcore->mops[mcore->cmop].type);
        }
    }

DONE:;
    /*printf("MCPPOP:    %zu\n", mcore->cmop); */
}


/*************************************************************************/
/*! This function frees all mops since the last push. This version is
    for poping the gkmcore and it uses free instead of gk_free.
 */
/*************************************************************************/
void gk_gkmcorePop(gk_mcore_t* mcore)
{
    while(mcore->cmop > 0)
    {
        mcore->cmop--;
        switch(mcore->mops[mcore->cmop].type)
        {
            case GK_MOPT_MARK: /* push marker */
                goto DONE;
                break;

            case GK_MOPT_HEAP: /* heap free */
                free(mcore->mops[mcore->cmop].ptr);
                mcore->cur_hallocs -= mcore->mops[mcore->cmop].nbytes;
                break;

            default:
                gk_errexit(SIGMEM,
                           "Unknown mop type of %d\n",
                           mcore->mops[mcore->cmop].type);
        }
    }

DONE:;
}


/*************************************************************************/
/*! Adds a memory allocation at the end of the list.
 */
/*************************************************************************/
void gk_mcoreAdd(gk_mcore_t* mcore, int type, size_t nbytes, void* ptr)
{
    if(mcore->cmop == mcore->nmops)
    {
        mcore->nmops *= 2;
        mcore->mops = (gk_mop_t*)realloc(mcore->mops, mcore->nmops * sizeof(gk_mop_t));
        if(mcore->mops == NULL)
            gk_errexit(SIGMEM, "***Memory allocation for gkmcore failed.\n");
    }

    mcore->mops[mcore->cmop].type   = type;
    mcore->mops[mcore->cmop].nbytes = nbytes;
    mcore->mops[mcore->cmop].ptr    = ptr;
    mcore->cmop++;

    switch(type)
    {
        case GK_MOPT_MARK:
            break;

        case GK_MOPT_CORE:
            mcore->num_callocs++;
            mcore->size_callocs += nbytes;
            mcore->cur_callocs += nbytes;
            if(mcore->max_callocs < mcore->cur_callocs)
                mcore->max_callocs = mcore->cur_callocs;
            break;

        case GK_MOPT_HEAP:
            mcore->num_hallocs++;
            mcore->size_hallocs += nbytes;
            mcore->cur_hallocs += nbytes;
            if(mcore->max_hallocs < mcore->cur_hallocs)
                mcore->max_hallocs = mcore->cur_hallocs;
            break;
        default:
            gk_errexit(SIGMEM, "Incorrect mcore type operation.\n");
    }
}


/*************************************************************************/
/*! Adds a memory allocation at the end of the list. This is the gkmcore
    version.
 */
/*************************************************************************/
void gk_gkmcoreAdd(gk_mcore_t* mcore, int type, size_t nbytes, void* ptr)
{
    if(mcore->cmop == mcore->nmops)
    {
        mcore->nmops *= 2;
        mcore->mops = (gk_mop_t*)realloc(mcore->mops, mcore->nmops * sizeof(gk_mop_t));
        if(mcore->mops == NULL)
            gk_errexit(SIGMEM, "***Memory allocation for gkmcore failed.\n");
    }

    mcore->mops[mcore->cmop].type   = type;
    mcore->mops[mcore->cmop].nbytes = nbytes;
    mcore->mops[mcore->cmop].ptr    = ptr;
    mcore->cmop++;

    switch(type)
    {
        case GK_MOPT_MARK:
            break;

        case GK_MOPT_HEAP:
            mcore->num_hallocs++;
            mcore->size_hallocs += nbytes;
            mcore->cur_hallocs += nbytes;
            if(mcore->max_hallocs < mcore->cur_hallocs)
                mcore->max_hallocs = mcore->cur_hallocs;
            break;
        default:
            gk_errexit(SIGMEM, "Incorrect mcore type operation.\n");
    }
}


/*************************************************************************/
/*! This function deletes the mop associated with the supplied pointer.
    The mop has to be a heap allocation, otherwise it fails violently.
 */
/*************************************************************************/
void gk_mcoreDel(gk_mcore_t* mcore, void* ptr)
{
    int i;

    for(i = mcore->cmop - 1; i >= 0; i--)
    {
        if(mcore->mops[i].type == GK_MOPT_MARK)
            gk_errexit(SIGMEM, "Could not find pointer %p in mcore\n", ptr);

        if(mcore->mops[i].ptr == ptr)
        {
            if(mcore->mops[i].type != GK_MOPT_HEAP)
                gk_errexit(SIGMEM, "Trying to delete a non-HEAP mop.\n");

            mcore->cur_hallocs -= mcore->mops[i].nbytes;
            mcore->mops[i] = mcore->mops[--mcore->cmop];
            return;
        }
    }

    gk_errexit(SIGMEM, "mcoreDel should never have been here!\n");
}


/*************************************************************************/
/*! This function deletes the mop associated with the supplied pointer.
    The mop has to be a heap allocation, otherwise it fails violently.
    This is the gkmcore version.
 */
/*************************************************************************/
void gk_gkmcoreDel(gk_mcore_t* mcore, void* ptr)
{
    int i;

    for(i = mcore->cmop - 1; i >= 0; i--)
    {
        if(mcore->mops[i].type == GK_MOPT_MARK)
            gk_errexit(SIGMEM, "Could not find pointer %p in mcore\n", ptr);

        if(mcore->mops[i].ptr == ptr)
        {
            if(mcore->mops[i].type != GK_MOPT_HEAP)
                gk_errexit(SIGMEM, "Trying to delete a non-HEAP mop.\n");

            mcore->cur_hallocs -= mcore->mops[i].nbytes;
            mcore->mops[i] = mcore->mops[--mcore->cmop];
            return;
        }
    }

    gk_errexit(SIGMEM, "gkmcoreDel should never have been here!\n");
}

/************************ memory.c ************************/
/*!
\file  memory.c
\brief This file contains various allocation routines

The allocation routines included are for 1D and 2D arrays of the
most datatypes that GKlib support. Many of these routines are
defined with the help of the macros in gk_memory.h. These macros
can be used to define other memory allocation routines.

\date   Started 4/3/2007
\author George
\version\verbatim $Id: memory.c 21050 2017-05-25 03:53:58Z karypis $ \endverbatim
*/


/* This is for the global mcore that tracks all heap allocations */
static __thread gk_mcore_t* gkmcore = NULL;


/*************************************************************************/
/*! Define the set of memory allocation routines for each data type */
/**************************************************************************/
GK_MKALLOC(gk_c, char)
GK_MKALLOC(gk_i, int)
GK_MKALLOC(gk_i32, int32_t)
GK_MKALLOC(gk_i64, int64_t)
GK_MKALLOC(gk_z, ssize_t)
GK_MKALLOC(gk_f, float)
GK_MKALLOC(gk_d, double)
GK_MKALLOC(gk_idx, gk_idx_t)

GK_MKALLOC(gk_ikv, gk_ikv_t)
GK_MKALLOC(gk_i32kv, gk_i32kv_t)
GK_MKALLOC(gk_i64kv, gk_i64kv_t)
GK_MKALLOC(gk_fkv, gk_fkv_t)
GK_MKALLOC(gk_dkv, gk_dkv_t)
GK_MKALLOC(gk_idxkv, gk_idxkv_t)


/*************************************************************************/
/*! This function allocates a two-dimensional matrix.
  */
/*************************************************************************/
void gk_AllocMatrix(void*** r_matrix, size_t elmlen, size_t ndim1, size_t ndim2)
{
    size_t i, j;
    void** matrix;

    *r_matrix = NULL;

    if((matrix = (void**)gk_malloc(ndim1 * sizeof(void*), "gk_AllocMatrix: matrix")) == NULL)
        return;

    for(i = 0; i < ndim1; i++)
    {
        if((matrix[i] = (void*)gk_malloc(ndim2 * elmlen, "gk_AllocMatrix: matrix[i]")) == NULL)
        {
            for(j = 0; j < i; j++)
                gk_free((void**)&matrix[j], LTERM);
            gk_free((void**)&matrix, LTERM);
            return;
        }
    }

    *r_matrix = matrix;
}


/*************************************************************************/
/*! This function frees a two-dimensional matrix.
  */
/*************************************************************************/
void gk_FreeMatrix(void*** r_matrix, size_t ndim1, size_t ndim2)
{
    size_t i;
    void** matrix;

    if((matrix = *r_matrix) == NULL)
        return;

    for(i = 0; i < ndim1; i++)
        gk_free((void**)&matrix[i], LTERM);

    gk_free((void**)r_matrix, LTERM);
}


/*************************************************************************/
/*! This function initializes tracking of heap allocations.
*/
/*************************************************************************/
int gk_malloc_init()
{
    if(gkmcore == NULL)
        gkmcore = gk_gkmcoreCreate();

    if(gkmcore == NULL)
        return 0;

    gk_gkmcorePush(gkmcore);

    return 1;
}


/*************************************************************************/
/*! This function frees the memory that has been allocated since the
    last call to gk_malloc_init().
*/
/*************************************************************************/
void gk_malloc_cleanup(int showstats)
{
    if(gkmcore != NULL)
    {
        gk_gkmcorePop(gkmcore);
        if(gkmcore->cmop == 0)
        {
            gk_gkmcoreDestroy(&gkmcore, showstats);
            gkmcore = NULL;
        }
    }
}


/*************************************************************************/
/*! This function is my wrapper around malloc that provides the following
    enhancements over malloc:
    * It always allocates one byte of memory, even if 0 bytes are requested.
      This is to ensure that checks of returned values do not lead to NULL
      due to 0 bytes requested.
    * It zeros-out the memory that is allocated. This is for a quick init
      of the underlying datastructures.
*/
/**************************************************************************/
void* gk_malloc(size_t nbytes, const char* msg)
{
    void* ptr = NULL;

    if(nbytes == 0)
        nbytes++; /* Force mallocs to actually allocate some memory */

    ptr = (void*)malloc(nbytes);

    if(ptr == NULL)
    {
        fprintf(stderr, "   Current memory used:  %10zu bytes\n", gk_GetCurMemoryUsed());
        fprintf(stderr, "   Maximum memory used:  %10zu bytes\n", gk_GetMaxMemoryUsed());
        gk_errexit(SIGMEM, "***Memory allocation failed for %s. Requested size: %zu bytes", msg, nbytes);
        return NULL;
    }

    /* add this memory allocation */
    if(gkmcore != NULL)
        gk_gkmcoreAdd(gkmcore, GK_MOPT_HEAP, nbytes, ptr);

    return ptr;
}


/*************************************************************************
* This function is my wrapper around realloc
**************************************************************************/
void* gk_realloc(void* oldptr, size_t nbytes, const char* msg)
{
    void* ptr = NULL;

    if(nbytes == 0)
        nbytes++; /* Force mallocs to actually allocate some memory */

    /* remove this memory de-allocation */
    if(gkmcore != NULL && oldptr != NULL)
        gk_gkmcoreDel(gkmcore, oldptr);

    ptr = (void*)realloc(oldptr, nbytes);

    if(ptr == NULL)
    {
        fprintf(stderr, "   Maximum memory used: %10zu bytes\n", gk_GetMaxMemoryUsed());
        fprintf(stderr, "   Current memory used: %10zu bytes\n", gk_GetCurMemoryUsed());
        gk_errexit(SIGMEM,
                   "***Memory realloc failed for %s. "
                   "Requested size: %zu bytes",
                   msg,
                   nbytes);
        return NULL;
    }

    /* add this memory allocation */
    if(gkmcore != NULL)
        gk_gkmcoreAdd(gkmcore, GK_MOPT_HEAP, nbytes, ptr);

    return ptr;
}


/*************************************************************************
* This function is my wrapper around free, allows multiple pointers
**************************************************************************/
void gk_free(void** ptr1, ...)
{
    va_list plist;
    void**  ptr;

    if(*ptr1 != NULL)
    {
        free(*ptr1);

        /* remove this memory de-allocation */
        if(gkmcore != NULL)
            gk_gkmcoreDel(gkmcore, *ptr1);
    }
    *ptr1 = NULL;

    va_start(plist, ptr1);
    while((ptr = va_arg(plist, void**)) != LTERM)
    {
        if(*ptr != NULL)
        {
            free(*ptr);

            /* remove this memory de-allocation */
            if(gkmcore != NULL)
                gk_gkmcoreDel(gkmcore, *ptr);
        }
        *ptr = NULL;
    }
    va_end(plist);
}


/*************************************************************************
* This function returns the current ammount of dynamically allocated
* memory that is used by the system
**************************************************************************/
size_t gk_GetCurMemoryUsed()
{
    if(gkmcore == NULL)
        return 0;
    else
        return gkmcore->cur_hallocs;
}


/*************************************************************************
* This function returns the maximum ammount of dynamically allocated
* memory that was used by the system
**************************************************************************/
size_t gk_GetMaxMemoryUsed()
{
    if(gkmcore == NULL)
        return 0;
    else
        return gkmcore->max_hallocs;
}


/************************ random.c ************************/
/*!
\file
\brief Various routines for providing portable 32 and 64 bit random number
       generators.

\date   Started 5/17/2007
\author George
\version\verbatim $Id: random.c 18796 2015-06-02 11:39:45Z karypis $ \endverbatim
*/


/*************************************************************************/
/*! Create the various random number functions */
/*************************************************************************/


/*************************************************************************/
/*! C-runtime random-number adapter used by the embedded METIS build. */
/*************************************************************************/
void gk_randinit(uint64_t seed)
{
    srand((unsigned int)seed);
}

/* Generates a random number from two C-runtime draws. */
uint64_t gk_randint64(void)
{
    return (uint64_t)(((uint64_t)rand()) << 32 | ((uint64_t)rand()));
}

/* generates a random number on [0, 2^32-1]-interval */
uint32_t gk_randint32(void)
{
    return (uint32_t)rand();
}


/*************************************************************************/
/*! Portable timer/file helpers needed by the retained METIS code. */
/*************************************************************************/
double gk_CPUSeconds(void)
{
    return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
}

gk_wclock_t gk_WClockSeconds(void)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

int gk_rmpath(char* path)
{
    return std::remove(path);
}
