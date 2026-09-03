/*
 * Derived from GKlib and adapted/reorganized for libuipc.
 * See LICENSE-GKlib in this directory.
 *
 * GKlib.h
 *
 * George's library of most frequently used routines
 *
 * $Id: GKlib.h 14866 2013-08-03 16:40:04Z karypis $
 *
 */

#ifndef _GKLIB_H_
#define _GKLIB_H_ 1

#define GKMSPACE

#if defined(_MSC_VER)
#define __MSC__
#ifndef __thread
#define __thread __declspec(thread)
#endif
#endif
#if defined(__ICC)
#define __ICC__
#endif

#if defined(__MSC__)
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <stdint.h>
#include <inttypes.h>
#endif


/*!
\file gk_arch.h
\brief This file contains various architecture-specific declerations

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_arch.h 21637 2018-01-03 22:37:24Z karypis $ \endverbatim
*/

#ifndef _GK_ARCH_H_
#define _GK_ARCH_H_

/*************************************************************************
* Architecture-specific differences in header files
**************************************************************************/
#ifdef LINUX
#if !defined(__USE_XOPEN)
#define __USE_XOPEN
#endif
#if !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE 600
#endif
#if !defined(__USE_XOPEN2K)
#define __USE_XOPEN2K
#endif
#endif


#ifdef HAVE_EXECINFO_H
#include <execinfo.h>
#endif


#ifdef __MSC__
#include <cstddef>
#include <process.h>
#include <sys/stat.h>
#include <windows.h>

using pid_t   = DWORD;
using ssize_t = std::ptrdiff_t;
#else
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#ifndef SUNOS
#include <stdint.h>
#endif
#include <inttypes.h>
#include <sys/types.h>
#ifndef __MINGW32__
#include <sys/resource.h>
#endif
#include <sys/time.h>
#include <unistd.h>
#endif

/*************************************************************************
* Architecture-specific modifications
**************************************************************************/
#ifdef SUNOS
#define PTRDIFF_MAX INT64_MAX
#endif

/* Modern MSVC and C++ <cmath> provide INFINITY; do not redefine it. */

#endif

/*************************************************************************
* Header file inclusion section
**************************************************************************/
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <memory.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include <signal.h>
#include <setjmp.h>
#include <assert.h>
#include <sys/stat.h>

/* Regex support is omitted: the embedded METIS path does not use GKlib's
 * regular-expression APIs. */

#if defined(__OPENMP__)
#include <omp.h>
#endif


/*!
\file  gk_types.h
\brief This file contains basic scalar datatype used in GKlib

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_types.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_TYPES_H_
#define _GK_TYPES_H_

/*************************************************************************
* Basic data type definitions. These definitions allow GKlib to separate
* the following elemental types:
* - loop iterator variables, which are set to size_t
* - signed and unsigned int variables that can be set to any # of bits
* - signed and unsigned long variables that can be set to any # of bits
* - real variables, which can be set to single or double precision.
**************************************************************************/
/*typedef ptrdiff_t       gk_idx_t;       */ /* index variable */
typedef ssize_t  gk_idx_t;                   /* index variable */
typedef int32_t  gk_int_t;                   /* integer values */
typedef uint32_t gk_uint_t;                  /* unsigned integer values */
typedef int64_t  gk_long_t;                  /* long integer values */
typedef uint64_t gk_ulong_t;                 /* unsigned long integer values */
typedef float    gk_real_t;                  /* real type */
typedef double   gk_dreal_t;                 /* double precission real type */
typedef double   gk_wclock_t;                /* wall-clock time */

/*#define GK_IDX_MAX PTRDIFF_MAX*/
#define GK_IDX_MAX ((SIZE_MAX >> 1) - 2)

#define PRIGKIDX "zd"
#define SCNGKIDX "zd"


#endif

/*!
\file gk_struct.h
\brief This file contains various datastructures used/provided by GKlib

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_struct.h 21988 2018-04-16 00:11:19Z karypis $ \endverbatim
*/

#ifndef _GK_STRUCT_H_
#define _GK_STRUCT_H_


/********************************************************************/
/*! Generator for gk_??KeyVal_t data structure */
/********************************************************************/
#define GK_MKKEYVALUE_T(NAME, KEYTYPE, VALTYPE)                                \
    typedef struct                                                             \
    {                                                                          \
        KEYTYPE key;                                                           \
        VALTYPE val;                                                           \
    } NAME;

/* The actual KeyVal data structures */
GK_MKKEYVALUE_T(gk_ckv_t, char, ssize_t)
GK_MKKEYVALUE_T(gk_ikv_t, int, ssize_t)
GK_MKKEYVALUE_T(gk_i8kv_t, int8_t, ssize_t)
GK_MKKEYVALUE_T(gk_i16kv_t, int16_t, ssize_t)
GK_MKKEYVALUE_T(gk_i32kv_t, int32_t, ssize_t)
GK_MKKEYVALUE_T(gk_i64kv_t, int64_t, ssize_t)
GK_MKKEYVALUE_T(gk_zkv_t, ssize_t, ssize_t)
GK_MKKEYVALUE_T(gk_zukv_t, size_t, ssize_t)
GK_MKKEYVALUE_T(gk_fkv_t, float, ssize_t)
GK_MKKEYVALUE_T(gk_dkv_t, double, ssize_t)
GK_MKKEYVALUE_T(gk_skv_t, char*, ssize_t)
GK_MKKEYVALUE_T(gk_idxkv_t, gk_idx_t, gk_idx_t)


/********************************************************************/
/*! Generator for gk_?pq_t data structure */
/********************************************************************/
#define GK_MKPQUEUE_T(NAME, KVTYPE)                                            \
    typedef struct                                                             \
    {                                                                          \
        size_t nnodes;                                                         \
        size_t maxnodes;                                                       \
                                                                               \
        /* Heap version of the data structure */                               \
        KVTYPE*  heap;                                                         \
        ssize_t* locator;                                                      \
    } NAME;

GK_MKPQUEUE_T(gk_ipq_t, gk_ikv_t)
GK_MKPQUEUE_T(gk_i32pq_t, gk_i32kv_t)
GK_MKPQUEUE_T(gk_i64pq_t, gk_i64kv_t)
GK_MKPQUEUE_T(gk_fpq_t, gk_fkv_t)
GK_MKPQUEUE_T(gk_dpq_t, gk_dkv_t)
GK_MKPQUEUE_T(gk_idxpq_t, gk_idxkv_t)


#define GK_MKPQUEUE2_T(NAME, KTYPE, VTYPE)                                     \
    typedef struct                                                             \
    {                                                                          \
        ssize_t nnodes;                                                        \
        ssize_t maxnodes;                                                      \
                                                                               \
        /* Heap version of the data structure */                               \
        KTYPE* keys;                                                           \
        VTYPE* vals;                                                           \
    } NAME;


/*-------------------------------------------------------------
 * The following data structure stores a sparse CSR format
 *-------------------------------------------------------------*/
typedef struct gk_csr_t
{
    int32_t  nrows, ncols;
    ssize_t *rowptr, *colptr;
    int32_t *rowind, *colind;
    int32_t *rowids, *colids;
    int32_t *rlabels, *clabels;
    int32_t *rmap, *cmap;
    float *  rowval, *colval;
    float *  rnorms, *cnorms;
    float *  rsums, *csums;
    float *  rsizes, *csizes;
    float *  rvols, *cvols;
    float *  rwgts, *cwgts;
} gk_csr_t;


/*-------------------------------------------------------------
 * The following data structure stores a sparse graph
 *-------------------------------------------------------------*/
typedef struct gk_graph_t
{
    int32_t  nvtxs;   /*!< The number of vertices in the graph */
    ssize_t* xadj;    /*!< The ptr-structure of the adjncy list */
    int32_t* adjncy;  /*!< The adjacency list of the graph */
    int32_t* iadjwgt; /*!< The integer edge weights */
    float*   fadjwgt; /*!< The floating point edge weights */
    int32_t* ivwgts;  /*!< The integer vertex weights */
    float*   fvwgts;  /*!< The floating point vertex weights */
    int32_t* ivsizes; /*!< The integer vertex sizes */
    float*   fvsizes; /*!< The floating point vertex sizes */
    int32_t* vlabels; /*!< The labels of the vertices */
} gk_graph_t;


/*-------------------------------------------------------------
 * The following data structure stores stores a string as a
 * pair of its allocated buffer and the buffer itself.
 *-------------------------------------------------------------*/
typedef struct gk_str_t
{
    size_t len;
    char*  buf;
} gk_str_t;


/*-------------------------------------------------------------
* The following data structure implements a string-2-int mapping
* table used for parsing command-line options
*-------------------------------------------------------------*/
typedef struct gk_StringMap_t
{
    char* name;
    int   id;
} gk_StringMap_t;


/*------------------------------------------------------------
 * This structure implements a simple hash table
 *------------------------------------------------------------*/
typedef struct gk_HTable_t
{
    int       nelements; /* The overall size of the hash-table */
    int       htsize;    /* The current size of the hash-table */
    gk_ikv_t* harray;    /* The actual hash-table */
} gk_HTable_t;


/*------------------------------------------------------------
 * This structure implements a gk_Tokens_t list returned by the
 * string tokenizer
 *------------------------------------------------------------*/
typedef struct gk_Tokens_t
{
    int    ntoks;  /* The number of tokens in the input string */
    char*  strbuf; /* The memory that stores all the entries */
    char** list;   /* Pointers to the strbuf for each element */
} gk_Tokens_t;


/*------------------------------------------------------------
 * This structure implements storage for an atom in a pdb file
 *------------------------------------------------------------*/
typedef struct atom
{
    int    serial;
    char*  name;
    char   altLoc;
    char*  resname;
    char   chainid;
    int    rserial;
    char   icode;
    char   element;
    double x;
    double y;
    double z;
    double opcy;
    double tmpt;
} atom;


/*------------------------------------------------------------
 * This structure implements storage for a center of mass for
 * a single residue.
 *------------------------------------------------------------*/
typedef struct center_of_mass
{
    char   name;
    double x;
    double y;
    double z;
} center_of_mass;


/*------------------------------------------------------------
 * This structure implements storage for a pdb protein
 *------------------------------------------------------------*/
typedef struct pdbf
{
    int             natoms;    /* Number of atoms */
    int             nresidues; /* Number of residues based on coordinates */
    int             ncas;
    int             nbbs;
    int             corruption;
    char*           resSeq;      /* Residue sequence based on coordinates    */
    char**          threeresSeq; /* three-letter residue sequence */
    atom*           atoms;
    atom**          bbs;
    atom**          cas;
    center_of_mass* cm;
} pdbf;


/*************************************************************
* Localization Structures for converting characters to integers
**************************************************************/
typedef struct gk_i2cc2i_t
{
    int   n;
    char* i2c;
    int*  c2i;
} gk_i2cc2i_t;


/*******************************************************************
 *This structure implements storage of a protein sequence
 * *****************************************************************/
typedef struct gk_seq_t
{

    int  len;      /*Number of Residues */
    int* sequence; /* Stores the sequence*/


    int** pssm; /* Stores the pssm matrix */
    int** psfm; /* Stores the psfm matrix */
    char* name; /* Stores the name of the sequence */

    int nsymbols;


} gk_seq_t;


/*************************************************************************/
/*! The following data structure stores information about a memory
    allocation operation that can either be served from gk_mcore_t or by
    a gk_malloc if not sufficient workspace memory is available. */
/*************************************************************************/
typedef struct gk_mop_t
{
    int     type;
    ssize_t nbytes;
    void*   ptr;
} gk_mop_t;


/*************************************************************************/
/*! The following structure defines the mcore for GKlib's customized
    memory allocations. */
/*************************************************************************/
typedef struct gk_mcore_t
{
    /* Workspace information */
    size_t coresize; /*!< The amount of core memory that has been allocated */
    size_t corecpos; /*!< Index of the first free location in core */
    void*  core;     /*!< Pointer to the core itself */

    /* These are for implementing a stack-based allocation scheme using both
     core and also dynamically allocated memory */
    size_t nmops; /*!< The number of maop_t entries that have been allocated */
    size_t cmop;  /*!< Index of the first free location in maops */
    gk_mop_t* mops; /*!< The array recording the maop_t operations */

    /* These are for keeping various statistics for wspacemalloc */
    size_t num_callocs;  /*!< The number of core mallocs */
    size_t num_hallocs;  /*!< The number of heap mallocs */
    size_t size_callocs; /*!< The total # of bytes in core mallocs */
    size_t size_hallocs; /*!< The total # of bytes in heap mallocs */
    size_t cur_callocs;  /*!< The current # of bytes in core mallocs */
    size_t cur_hallocs;  /*!< The current # of bytes in heap mallocs */
    size_t max_callocs; /*!< The maximum # of bytes in core mallocs at any given time */
    size_t max_hallocs; /*!< The maximum # of bytes in heap mallocs at any given time */

} gk_mcore_t;


/*************************************************************************/
/*! The following structure is used for cache simulation for performance
    modeling and analysis. */
/*************************************************************************/
typedef struct gk_cache_t
{
    /*! The total cache is nway*(2^(cnbits+lnbits)) bytes */
    uint32_t nway;   /*!< the associativity of the cache */
    uint32_t lnbits; /*!< the number of address bits indexing the cache line */
    uint32_t cnbits; /*!< the number of address bits indexing the cache */
    size_t   csize;  /*!< 2^cnbits */
    size_t   cmask;  /*!< csize-1 */

    uint64_t clock; /*!< a clock in terms of accesses */

    uint64_t* latimes; /*!< a cacheline-level last access time */
    size_t*   clines;  /*!< the cache in terms of cachelines */

    uint64_t nhits;   /*!< counts the number of hits */
    uint64_t nmisses; /*!< counts the number of misses */
} gk_cache_t;


#endif

/*!
\file gk_externs.h
\brief This file contains definitions of external variables created by GKlib

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_externs.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_EXTERNS_H_
#define _GK_EXTERNS_H_


/*************************************************************************
* Extern variable definition. Hopefully, the __thread makes them thread-safe.
**************************************************************************/
#ifndef _GK_ERROR_C_
/* declared in error.c */
extern __thread int     gk_cur_jbufs;
extern __thread jmp_buf gk_jbufs[];
extern __thread jmp_buf gk_jbuf;

#endif

#endif

/*!
\file gk_defs.h
\brief This file contains various constants definitions

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_defs.h 22039 2018-05-26 16:34:48Z karypis $ \endverbatim
*/

#ifndef _GK_DEFS_H_
#define _GK_DEFS_H_


#define LTERM (void**)0 /* List terminator for GKfree() */

/* mopt_t types */
#define GK_MOPT_MARK 1
#define GK_MOPT_CORE 2
#define GK_MOPT_HEAP 3

#define HTABLE_EMPTY -1
#define HTABLE_DELETED -2
#define HTABLE_FIRST 1
#define HTABLE_NEXT 2

/* pdb corruption bit switches */
#define CRP_ALTLOCS 1
#define CRP_MISSINGCA 2
#define CRP_MISSINGBB 4
#define CRP_MULTICHAIN 8
#define CRP_MULTICA 16
#define CRP_MULTIBB 32

#define MAXLINELEN 300000

/* GKlib signals to standard signal mapping */
#define SIGMEM SIGABRT
#define SIGERR SIGTERM


/* CSR-related defines */
#define GK_CSR_ROW 1
#define GK_CSR_COL 2
#define GK_CSR_ROWCOL 3

#define GK_CSR_MAXTF 1
#define GK_CSR_SQRT 2
#define GK_CSR_POW25 3
#define GK_CSR_POW65 4
#define GK_CSR_POW75 5
#define GK_CSR_POW85 6
#define GK_CSR_LOG 7
#define GK_CSR_IDF 8
#define GK_CSR_IDF2 9
#define GK_CSR_MAXTF2 10

#define GK_CSR_DOTP 1
#define GK_CSR_COS 2
#define GK_CSR_JAC 3
#define GK_CSR_MIN 4
#define GK_CSR_AMIN 5

#define GK_CSR_FMT_AUTO 2
#define GK_CSR_FMT_CLUTO 1
#define GK_CSR_FMT_CSR 2
#define GK_CSR_FMT_METIS 3
#define GK_CSR_FMT_BINROW 4
#define GK_CSR_FMT_BINCOL 5
#define GK_CSR_FMT_IJV 6
#define GK_CSR_FMT_BIJV 7

#define GK_CSR_SYM_SUM 1
#define GK_CSR_SYM_MIN 2
#define GK_CSR_SYM_MAX 3
#define GK_CSR_SYM_AVG 4


#define GK_GRAPH_FMT_METIS 1
#define GK_GRAPH_FMT_IJV 2
#define GK_GRAPH_FMT_HIJV 3

#define GK_GRAPH_SYM_SUM 1
#define GK_GRAPH_SYM_MIN 2
#define GK_GRAPH_SYM_MAX 3
#define GK_GRAPH_SYM_AVG 4

#endif

/*!
\file gk_macros.h
\brief This file contains various macros

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_macros.h 15048 2013-08-31 19:38:14Z karypis $ \endverbatim
*/

#ifndef _GK_MACROS_H_
#define _GK_MACROS_H_

/*-------------------------------------------------------------
 * Usefull commands
 *-------------------------------------------------------------*/
#define gk_max(a, b) ((a) >= (b) ? (a) : (b))
#define gk_min(a, b) ((a) >= (b) ? (b) : (a))
#define gk_max3(a, b, c)                                                       \
    ((a) >= (b) && (a) >= (c) ? (a) : ((b) >= (a) && (b) >= (c) ? (b) : (c)))
#define gk_SWAP(a, b, tmp)                                                     \
    do                                                                         \
    {                                                                          \
        (tmp) = (a);                                                           \
        (a)   = (b);                                                           \
        (b)   = (tmp);                                                         \
    } while(0)
#define INC_DEC(a, b, val)                                                     \
    do                                                                         \
    {                                                                          \
        (a) += (val);                                                          \
        (b) -= (val);                                                          \
    } while(0)
#define sign(a, b) ((a >= 0 ? b : -b))

#define ONEOVERRANDMAX (1.0 / (RAND_MAX + 1.0))
#define RandomInRange(u) ((int)(ONEOVERRANDMAX * (u) * rand()))
#define RandomInRange_r(s, u) ((int)(ONEOVERRANDMAX * (u) * rand_r(s)))

#define gk_abs(x) ((x) >= 0 ? (x) : -(x))


/*-------------------------------------------------------------
 * Timing macros
 *-------------------------------------------------------------*/
#define gk_clearcputimer(tmr) (tmr = 0.0)
#define gk_startcputimer(tmr) (tmr -= gk_CPUSeconds())
#define gk_stopcputimer(tmr) (tmr += gk_CPUSeconds())
#define gk_getcputimer(tmr) (tmr)

#define gk_clearwctimer(tmr) (tmr = 0.0)
#define gk_startwctimer(tmr) (tmr -= gk_WClockSeconds())
#define gk_stopwctimer(tmr) (tmr += gk_WClockSeconds())
#define gk_getwctimer(tmr) (tmr)

/*-------------------------------------------------------------
 * dbglvl handling macros
 *-------------------------------------------------------------*/
#define IFSET(a, flag, cmd)                                                    \
    if((a) & (flag))                                                           \
        (cmd);


/*-------------------------------------------------------------
 * gracefull library exit macro
 *-------------------------------------------------------------*/
#define GKSETJMP() (setjmp(gk_return_to_entry))
#define gk_sigcatch() (setjmp(gk_jbufs[gk_cur_jbufs]))


/*-------------------------------------------------------------
 * Debuging memory leaks
 *-------------------------------------------------------------*/
#ifdef DMALLOC
#define MALLOC_CHECK(ptr)                                                                       \
    if(malloc_verify((ptr)) == DMALLOC_VERIFY_ERROR)                                            \
    {                                                                                           \
        printf("***MALLOC_CHECK failed on line %d of file %s: " #ptr "\n", __LINE__, __FILE__); \
        abort();                                                                                \
    }
#else
#define MALLOC_CHECK(ptr) ;
#endif


/*-------------------------------------------------------------
 * CSR conversion macros
 *-------------------------------------------------------------*/
#define MAKECSR(i, n, a)                                                       \
    do                                                                         \
    {                                                                          \
        for(i = 1; i < n; i++)                                                 \
            a[i] += a[i - 1];                                                  \
        for(i = n; i > 0; i--)                                                 \
            a[i] = a[i - 1];                                                   \
        a[0] = 0;                                                              \
    } while(0)

#define SHIFTCSR(i, n, a)                                                      \
    do                                                                         \
    {                                                                          \
        for(i = n; i > 0; i--)                                                 \
            a[i] = a[i - 1];                                                   \
        a[0] = 0;                                                              \
    } while(0)


/*-------------------------------------------------------------
 * ASSERTS that cannot be turned off!
 *-------------------------------------------------------------*/
#define GKASSERT(expr)                                                                        \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
        abort();                                                                              \
    }

#define GKASSERTP(expr, msg)                                                                  \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
        printf msg;                                                                           \
        printf("\n");                                                                         \
        abort();                                                                              \
    }

#define GKCUASSERT(expr)                                                                      \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
    }

#define GKWARN(expr)                                                                          \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
    }

#define GKCUASSERTP(expr, msg)                                                                \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
        printf msg;                                                                           \
        printf("\n");                                                                         \
    }

#define GKWARNP(expr, msg)                                                                    \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
        printf msg;                                                                           \
        printf("\n");                                                                         \
    }


/*-------------------------------------------------------------
 * Program Assertions
 *-------------------------------------------------------------*/
#ifndef NDEBUG
#define ASSERT(expr)                                                                          \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
        assert(expr);                                                                         \
    }

#define ASSERTP(expr, msg)                                                                    \
    if(!(expr))                                                                               \
    {                                                                                         \
        printf("***ASSERTION failed on line %d of file %s: " #expr "\n", __LINE__, __FILE__); \
        printf msg;                                                                           \
        printf("\n");                                                                         \
        assert(expr);                                                                         \
    }
#else
#define ASSERT(expr) ;
#define ASSERTP(expr, msg) ;
#endif

#ifndef NDEBUG2
#define ASSERT2 ASSERT
#define ASSERTP2 ASSERTP
#else
#define ASSERT2(expr) ;
#define ASSERTP2(expr, msg) ;
#endif


#endif

/* The unused GKlib command-line parser and its glibc-derived qsort
 * template are omitted. The deterministic C++ implementation supplies the
 * sorting wrappers in metis_instantiations.cpp. */

/*!
\file  gk_mkblas.h
\brief Templates for BLAS-like routines

\date   Started 3/28/07
\author George
\version\verbatim $Id: gk_mkblas.h 16304 2014-02-25 14:27:19Z karypis $ \endverbatim
*/

#ifndef _GK_MKBLAS_H_
#define _GK_MKBLAS_H_


#define GK_MKBLAS(PRFX, TYPE, OUTTYPE)                                                 \
    /*************************************************************************/        \
    /*! The macro for gk_?incset()-class of routines */                                \
    /*************************************************************************/        \
    TYPE* PRFX##incset(size_t n, TYPE baseval, TYPE* x)                                \
    {                                                                                  \
        size_t i;                                                                      \
                                                                                       \
        for(i = 0; i < n; i++)                                                         \
            x[i] = baseval + i;                                                        \
                                                                                       \
        return x;                                                                      \
    }                                                                                  \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?max()-class of routines */                                   \
    /*************************************************************************/        \
    TYPE PRFX##max(size_t n, TYPE* x, size_t incx)                                     \
    {                                                                                  \
        size_t i;                                                                      \
        TYPE   max;                                                                    \
                                                                                       \
        if(n <= 0)                                                                     \
            return (TYPE)0;                                                            \
                                                                                       \
        for(max = (*x), x += incx, i = 1; i < n; i++, x += incx)                       \
            max = ((*x) > max ? (*x) : max);                                           \
                                                                                       \
        return max;                                                                    \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?min()-class of routines */                                   \
    /*************************************************************************/        \
    TYPE PRFX##min(size_t n, TYPE* x, size_t incx)                                     \
    {                                                                                  \
        size_t i;                                                                      \
        TYPE   min;                                                                    \
                                                                                       \
        if(n <= 0)                                                                     \
            return (TYPE)0;                                                            \
                                                                                       \
        for(min = (*x), x += incx, i = 1; i < n; i++, x += incx)                       \
            min = ((*x) < min ? (*x) : min);                                           \
                                                                                       \
        return min;                                                                    \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?argmax()-class of routines */                                \
    /*************************************************************************/        \
    size_t PRFX##argmax(size_t n, TYPE* x, size_t incx)                                \
    {                                                                                  \
        size_t i, j, max = 0;                                                          \
                                                                                       \
        for(i = 1, j = incx; i < n; i++, j += incx)                                    \
            max = (x[j] > x[max] ? j : max);                                           \
                                                                                       \
        return (size_t)(max / incx);                                                   \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?argmin()-class of routines */                                \
    /*************************************************************************/        \
    size_t PRFX##argmin(size_t n, TYPE* x, size_t incx)                                \
    {                                                                                  \
        size_t i, j, min = 0;                                                          \
                                                                                       \
        for(i = 1, j = incx; i < n; i++, j += incx)                                    \
            min = (x[j] < x[min] ? j : min);                                           \
                                                                                       \
        return (size_t)(min / incx);                                                   \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?argmax_n()-class of routines */                              \
    /*************************************************************************/        \
    size_t PRFX##argmax_n(size_t n, TYPE* x, size_t incx, size_t k)                    \
    {                                                                                  \
        size_t      i, j, max_n;                                                       \
        PRFX##kv_t* cand;                                                              \
                                                                                       \
        cand = PRFX##kvmalloc(n, "GK_ARGMAX_N: cand");                                 \
                                                                                       \
        for(i = 0, j = 0; i < n; i++, j += incx)                                       \
        {                                                                              \
            cand[i].val = i;                                                           \
            cand[i].key = x[j];                                                        \
        }                                                                              \
        PRFX##kvsortd(n, cand);                                                        \
                                                                                       \
        max_n = cand[k - 1].val;                                                       \
                                                                                       \
        gk_free((void**)&cand, LTERM);                                                 \
                                                                                       \
        return max_n;                                                                  \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?sum()-class of routines */                                   \
    /**************************************************************************/       \
    OUTTYPE PRFX##sum(size_t n, TYPE* x, size_t incx)                                  \
    {                                                                                  \
        size_t  i;                                                                     \
        OUTTYPE sum = 0;                                                               \
                                                                                       \
        for(i = 0; i < n; i++, x += incx)                                              \
            sum += (*x);                                                               \
                                                                                       \
        return sum;                                                                    \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?scale()-class of routines */                                 \
    /**************************************************************************/       \
    TYPE* PRFX##scale(size_t n, TYPE alpha, TYPE* x, size_t incx)                      \
    {                                                                                  \
        size_t i;                                                                      \
                                                                                       \
        for(i = 0; i < n; i++, x += incx)                                              \
            (*x) *= alpha;                                                             \
                                                                                       \
        return x;                                                                      \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?norm2()-class of routines */                                 \
    /**************************************************************************/       \
    OUTTYPE PRFX##norm2(size_t n, TYPE* x, size_t incx)                                \
    {                                                                                  \
        size_t  i;                                                                     \
        OUTTYPE partial = 0;                                                           \
                                                                                       \
        for(i = 0; i < n; i++, x += incx)                                              \
            partial += (*x) * (*x);                                                    \
                                                                                       \
        return (partial > 0 ? (OUTTYPE)sqrt((double)partial) : (OUTTYPE)0);            \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?dot()-class of routines */                                   \
    /**************************************************************************/       \
    OUTTYPE PRFX##dot(size_t n, TYPE* x, size_t incx, TYPE* y, size_t incy)            \
    {                                                                                  \
        size_t  i;                                                                     \
        OUTTYPE partial = 0.0;                                                         \
                                                                                       \
        for(i = 0; i < n; i++, x += incx, y += incy)                                   \
            partial += (*x) * (*y);                                                    \
                                                                                       \
        return partial;                                                                \
    }                                                                                  \
                                                                                       \
                                                                                       \
    /*************************************************************************/        \
    /*! The macro for gk_?axpy()-class of routines */                                  \
    /**************************************************************************/       \
    TYPE* PRFX##axpy(size_t n, TYPE alpha, TYPE* x, size_t incx, TYPE* y, size_t incy) \
    {                                                                                  \
        size_t i;                                                                      \
        TYPE*  y_in = y;                                                               \
                                                                                       \
        for(i = 0; i < n; i++, x += incx, y += incy)                                   \
            *y += alpha * (*x);                                                        \
                                                                                       \
        return y_in;                                                                   \
    }


#define GK_MKBLAS_PROTO(PRFX, TYPE, OUTTYPE)                                   \
    TYPE*   PRFX##incset(size_t n, TYPE baseval, TYPE* x);                     \
    TYPE    PRFX##max(size_t n, TYPE* x, size_t incx);                         \
    TYPE    PRFX##min(size_t n, TYPE* x, size_t incx);                         \
    size_t  PRFX##argmax(size_t n, TYPE* x, size_t incx);                      \
    size_t  PRFX##argmin(size_t n, TYPE* x, size_t incx);                      \
    size_t  PRFX##argmax_n(size_t n, TYPE* x, size_t incx, size_t k);          \
    OUTTYPE PRFX##sum(size_t n, TYPE* x, size_t incx);                         \
    TYPE*   PRFX##scale(size_t n, TYPE alpha, TYPE* x, size_t incx);           \
    OUTTYPE PRFX##norm2(size_t n, TYPE* x, size_t incx);                       \
    OUTTYPE PRFX##dot(size_t n, TYPE* x, size_t incx, TYPE* y, size_t incy);   \
    TYPE* PRFX##axpy(size_t n, TYPE alpha, TYPE* x, size_t incx, TYPE* y, size_t incy);


#endif

/*!
\file  gk_mkmemory.h
\brief Templates for memory allocation routines

\date   Started 3/29/07
\author George
\version\verbatim $Id: gk_mkmemory.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_MKMEMORY_H_
#define _GK_MKMEMORY_H_


#define GK_MKALLOC(PRFX, TYPE)                                                           \
    /*************************************************************************/          \
    /*! The macro for gk_?malloc()-class of routines */                                  \
    /**************************************************************************/         \
    TYPE* PRFX##malloc(size_t n, const char* msg)                                        \
    {                                                                                    \
        return (TYPE*)gk_malloc(sizeof(TYPE) * n, msg);                                  \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?realloc()-class of routines */                                 \
    /**************************************************************************/         \
    TYPE* PRFX##realloc(TYPE* ptr, size_t n, const char* msg)                            \
    {                                                                                    \
        return (TYPE*)gk_realloc((void*)ptr, sizeof(TYPE) * n, msg);                     \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?smalloc()-class of routines */                                 \
    /**************************************************************************/         \
    TYPE* PRFX##smalloc(size_t n, TYPE ival, const char* msg)                            \
    {                                                                                    \
        TYPE* ptr;                                                                       \
                                                                                         \
        ptr = (TYPE*)gk_malloc(sizeof(TYPE) * n, msg);                                   \
        if(ptr == NULL)                                                                  \
            return NULL;                                                                 \
                                                                                         \
        return PRFX##set(n, ival, ptr);                                                  \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?set()-class of routines */                                     \
    /*************************************************************************/          \
    TYPE* PRFX##set(size_t n, TYPE val, TYPE* x)                                         \
    {                                                                                    \
        size_t i;                                                                        \
                                                                                         \
        for(i = 0; i < n; i++)                                                           \
            x[i] = val;                                                                  \
                                                                                         \
        return x;                                                                        \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?set()-class of routines */                                     \
    /*************************************************************************/          \
    TYPE* PRFX##copy(size_t n, TYPE* a, TYPE* b)                                         \
    {                                                                                    \
        return (TYPE*)memmove((void*)b, (void*)a, sizeof(TYPE) * n);                     \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?AllocMatrix()-class of routines */                             \
    /**************************************************************************/         \
    TYPE** PRFX##AllocMatrix(size_t ndim1, size_t ndim2, TYPE value, const char* errmsg) \
    {                                                                                    \
        gk_idx_t i, j;                                                                   \
        TYPE**   matrix;                                                                 \
                                                                                         \
        matrix = (TYPE**)gk_malloc(ndim1 * sizeof(TYPE*), errmsg);                       \
        if(matrix == NULL)                                                               \
            return NULL;                                                                 \
                                                                                         \
        for(i = 0; i < ndim1; i++)                                                       \
        {                                                                                \
            matrix[i] = PRFX##smalloc(ndim2, value, errmsg);                             \
            if(matrix[i] == NULL)                                                        \
            {                                                                            \
                for(j = 0; j < i; j++)                                                   \
                    gk_free((void**)&matrix[j], LTERM);                                  \
                return NULL;                                                             \
            }                                                                            \
        }                                                                                \
                                                                                         \
        return matrix;                                                                   \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?AllocMatrix()-class of routines */                             \
    /**************************************************************************/         \
    void PRFX##FreeMatrix(TYPE*** r_matrix, size_t ndim1, size_t ndim2)                  \
    {                                                                                    \
        gk_idx_t i;                                                                      \
        TYPE**   matrix;                                                                 \
                                                                                         \
        if(*r_matrix == NULL)                                                            \
            return;                                                                      \
                                                                                         \
        matrix = *r_matrix;                                                              \
                                                                                         \
        for(i = 0; i < ndim1; i++)                                                       \
            gk_free((void**)&(matrix[i]), LTERM);                                        \
                                                                                         \
        gk_free((void**)r_matrix, LTERM);                                                \
    }                                                                                    \
                                                                                         \
                                                                                         \
    /*************************************************************************/          \
    /*! The macro for gk_?SetMatrix()-class of routines */                               \
    /**************************************************************************/         \
    void PRFX##SetMatrix(TYPE** matrix, size_t ndim1, size_t ndim2, TYPE value)          \
    {                                                                                    \
        gk_idx_t i, j;                                                                   \
                                                                                         \
        for(i = 0; i < ndim1; i++)                                                       \
        {                                                                                \
            for(j = 0; j < ndim2; j++)                                                   \
                matrix[i][j] = value;                                                    \
        }                                                                                \
    }


#define GK_MKALLOC_PROTO(PRFX, TYPE)                                                      \
    TYPE* PRFX##malloc(size_t n, const char* msg);                                        \
    TYPE* PRFX##realloc(TYPE* ptr, size_t n, const char* msg);                            \
    TYPE* PRFX##smalloc(size_t n, TYPE ival, const char* msg);                            \
    TYPE* PRFX##set(size_t n, TYPE val, TYPE* x);                                         \
    TYPE* PRFX##copy(size_t n, TYPE* a, TYPE* b);                                         \
    TYPE** PRFX##AllocMatrix(size_t ndim1, size_t ndim2, TYPE value, const char* errmsg); \
    void PRFX##FreeMatrix(TYPE*** r_matrix, size_t ndim1, size_t ndim2);                  \
    void PRFX##SetMatrix(TYPE** matrix, size_t ndim1, size_t ndim2, TYPE value);


#endif

/*!
\file  gk_mkpqueue.h
\brief Templates for priority queues

\date   Started 4/09/07
\author George
\version\verbatim $Id: gk_mkpqueue.h 21742 2018-01-26 16:59:15Z karypis $ \endverbatim
*/


#ifndef _GK_MKPQUEUE_H
#define _GK_MKPQUEUE_H


#define GK_MKPQUEUE(FPRFX, PQT, KVT, KT, VT, KVMALLOC, KMAX, KEY_LT)                  \
    /*************************************************************************/       \
    /*! This function creates and initializes a priority queue */                     \
    /**************************************************************************/      \
    PQT* FPRFX##Create(size_t maxnodes)                                               \
    {                                                                                 \
        PQT* queue;                                                                   \
                                                                                      \
        queue = (PQT*)gk_malloc(sizeof(PQT), "gk_pqCreate: queue");                   \
        FPRFX##Init(queue, maxnodes);                                                 \
                                                                                      \
        return queue;                                                                 \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function initializes the data structures of the priority queue */        \
    /**************************************************************************/      \
    void FPRFX##Init(PQT* queue, size_t maxnodes)                                     \
    {                                                                                 \
        queue->nnodes   = 0;                                                          \
        queue->maxnodes = maxnodes;                                                   \
                                                                                      \
        queue->heap    = KVMALLOC(maxnodes, "gk_PQInit: heap");                       \
        queue->locator = gk_idxsmalloc(maxnodes, -1, "gk_PQInit: locator");           \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function resets the priority queue */                                    \
    /**************************************************************************/      \
    void FPRFX##Reset(PQT* queue)                                                     \
    {                                                                                 \
        ssize_t  i;                                                                   \
        ssize_t* locator = queue->locator;                                            \
        KVT*     heap    = queue->heap;                                               \
                                                                                      \
        for(i = queue->nnodes - 1; i >= 0; i--)                                       \
            locator[heap[i].val] = -1;                                                \
        queue->nnodes = 0;                                                            \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function frees the internal datastructures of the priority queue */      \
    /**************************************************************************/      \
    void FPRFX##Free(PQT* queue)                                                      \
    {                                                                                 \
        if(queue == NULL)                                                             \
            return;                                                                   \
        gk_free((void**)&queue->heap, &queue->locator, LTERM);                        \
        queue->maxnodes = 0;                                                          \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function frees the internal datastructures of the priority queue \
    and the queue itself */       \
    /**************************************************************************/      \
    void FPRFX##Destroy(PQT* queue)                                                   \
    {                                                                                 \
        if(queue == NULL)                                                             \
            return;                                                                   \
        FPRFX##Free(queue);                                                           \
        gk_free((void**)&queue, LTERM);                                               \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function returns the length of the queue */                              \
    /**************************************************************************/      \
    size_t FPRFX##Length(PQT* queue)                                                  \
    {                                                                                 \
        return queue->nnodes;                                                         \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function adds an item in the priority queue */                           \
    /**************************************************************************/      \
    int FPRFX##Insert(PQT* queue, VT node, KT key)                                    \
    {                                                                                 \
        ssize_t  i, j;                                                                \
        ssize_t* locator = queue->locator;                                            \
        KVT*     heap    = queue->heap;                                               \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        ASSERT(locator[node] == -1);                                                  \
                                                                                      \
        i = queue->nnodes++;                                                          \
        while(i > 0)                                                                  \
        {                                                                             \
            j = (i - 1) >> 1;                                                         \
            if(KEY_LT(key, heap[j].key))                                              \
            {                                                                         \
                heap[i]              = heap[j];                                       \
                locator[heap[i].val] = i;                                             \
                i                    = j;                                             \
            }                                                                         \
            else                                                                      \
                break;                                                                \
        }                                                                             \
        ASSERT(i >= 0);                                                               \
        heap[i].key   = key;                                                          \
        heap[i].val   = node;                                                         \
        locator[node] = i;                                                            \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        return 0;                                                                     \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function deletes an item from the priority queue */                      \
    /**************************************************************************/      \
    int FPRFX##Delete(PQT* queue, VT node)                                            \
    {                                                                                 \
        ssize_t  i, j;                                                                \
        size_t   nnodes;                                                              \
        KT       newkey, oldkey;                                                      \
        ssize_t* locator = queue->locator;                                            \
        KVT*     heap    = queue->heap;                                               \
                                                                                      \
        ASSERT(locator[node] != -1);                                                  \
        ASSERT(heap[locator[node]].val == node);                                      \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        i             = locator[node];                                                \
        locator[node] = -1;                                                           \
                                                                                      \
        if(--queue->nnodes > 0 && heap[queue->nnodes].val != node)                    \
        {                                                                             \
            node   = heap[queue->nnodes].val;                                         \
            newkey = heap[queue->nnodes].key;                                         \
            oldkey = heap[i].key;                                                     \
                                                                                      \
            if(KEY_LT(newkey, oldkey))                                                \
            { /* Filter-up */                                                         \
                while(i > 0)                                                          \
                {                                                                     \
                    j = (i - 1) >> 1;                                                 \
                    if(KEY_LT(newkey, heap[j].key))                                   \
                    {                                                                 \
                        heap[i]              = heap[j];                               \
                        locator[heap[i].val] = i;                                     \
                        i                    = j;                                     \
                    }                                                                 \
                    else                                                              \
                        break;                                                        \
                }                                                                     \
            }                                                                         \
            else                                                                      \
            { /* Filter down */                                                       \
                nnodes = queue->nnodes;                                               \
                while((j = (i << 1) + 1) < nnodes)                                    \
                {                                                                     \
                    if(KEY_LT(heap[j].key, newkey))                                   \
                    {                                                                 \
                        if(j + 1 < nnodes && KEY_LT(heap[j + 1].key, heap[j].key))    \
                            j++;                                                      \
                        heap[i]              = heap[j];                               \
                        locator[heap[i].val] = i;                                     \
                        i                    = j;                                     \
                    }                                                                 \
                    else if(j + 1 < nnodes && KEY_LT(heap[j + 1].key, newkey))        \
                    {                                                                 \
                        j++;                                                          \
                        heap[i]              = heap[j];                               \
                        locator[heap[i].val] = i;                                     \
                        i                    = j;                                     \
                    }                                                                 \
                    else                                                              \
                        break;                                                        \
                }                                                                     \
            }                                                                         \
                                                                                      \
            heap[i].key   = newkey;                                                   \
            heap[i].val   = node;                                                     \
            locator[node] = i;                                                        \
        }                                                                             \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        return 0;                                                                     \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function updates the key values associated for a particular item */      \
    /**************************************************************************/      \
    void FPRFX##Update(PQT* queue, VT node, KT newkey)                                \
    {                                                                                 \
        ssize_t  i, j;                                                                \
        size_t   nnodes;                                                              \
        KT       oldkey;                                                              \
        ssize_t* locator = queue->locator;                                            \
        KVT*     heap    = queue->heap;                                               \
                                                                                      \
        oldkey = heap[locator[node]].key;                                             \
        if(!KEY_LT(newkey, oldkey) && !KEY_LT(oldkey, newkey))                        \
            return;                                                                   \
                                                                                      \
        ASSERT(locator[node] != -1);                                                  \
        ASSERT(heap[locator[node]].val == node);                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        i = locator[node];                                                            \
                                                                                      \
        if(KEY_LT(newkey, oldkey))                                                    \
        { /* Filter-up */                                                             \
            while(i > 0)                                                              \
            {                                                                         \
                j = (i - 1) >> 1;                                                     \
                if(KEY_LT(newkey, heap[j].key))                                       \
                {                                                                     \
                    heap[i]              = heap[j];                                   \
                    locator[heap[i].val] = i;                                         \
                    i                    = j;                                         \
                }                                                                     \
                else                                                                  \
                    break;                                                            \
            }                                                                         \
        }                                                                             \
        else                                                                          \
        { /* Filter down */                                                           \
            nnodes = queue->nnodes;                                                   \
            while((j = (i << 1) + 1) < nnodes)                                        \
            {                                                                         \
                if(KEY_LT(heap[j].key, newkey))                                       \
                {                                                                     \
                    if(j + 1 < nnodes && KEY_LT(heap[j + 1].key, heap[j].key))        \
                        j++;                                                          \
                    heap[i]              = heap[j];                                   \
                    locator[heap[i].val] = i;                                         \
                    i                    = j;                                         \
                }                                                                     \
                else if(j + 1 < nnodes && KEY_LT(heap[j + 1].key, newkey))            \
                {                                                                     \
                    j++;                                                              \
                    heap[i]              = heap[j];                                   \
                    locator[heap[i].val] = i;                                         \
                    i                    = j;                                         \
                }                                                                     \
                else                                                                  \
                    break;                                                            \
            }                                                                         \
        }                                                                             \
                                                                                      \
        heap[i].key   = newkey;                                                       \
        heap[i].val   = node;                                                         \
        locator[node] = i;                                                            \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        return;                                                                       \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function returns the item at the top of the queue and removes\
    it from the priority queue */           \
    /**************************************************************************/      \
    VT FPRFX##GetTop(PQT* queue)                                                      \
    {                                                                                 \
        ssize_t  i, j;                                                                \
        ssize_t* locator;                                                             \
        KVT*     heap;                                                                \
        VT       vtx, node;                                                           \
        KT       key;                                                                 \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
                                                                                      \
        if(queue->nnodes == 0)                                                        \
            return -1;                                                                \
                                                                                      \
        queue->nnodes--;                                                              \
                                                                                      \
        heap    = queue->heap;                                                        \
        locator = queue->locator;                                                     \
                                                                                      \
        vtx          = heap[0].val;                                                   \
        locator[vtx] = -1;                                                            \
                                                                                      \
        if((i = queue->nnodes) > 0)                                                   \
        {                                                                             \
            key  = heap[i].key;                                                       \
            node = heap[i].val;                                                       \
            i    = 0;                                                                 \
            while((j = 2 * i + 1) < queue->nnodes)                                    \
            {                                                                         \
                if(KEY_LT(heap[j].key, key))                                          \
                {                                                                     \
                    if(j + 1 < queue->nnodes && KEY_LT(heap[j + 1].key, heap[j].key)) \
                        j = j + 1;                                                    \
                    heap[i]              = heap[j];                                   \
                    locator[heap[i].val] = i;                                         \
                    i                    = j;                                         \
                }                                                                     \
                else if(j + 1 < queue->nnodes && KEY_LT(heap[j + 1].key, key))        \
                {                                                                     \
                    j                    = j + 1;                                     \
                    heap[i]              = heap[j];                                   \
                    locator[heap[i].val] = i;                                         \
                    i                    = j;                                         \
                }                                                                     \
                else                                                                  \
                    break;                                                            \
            }                                                                         \
                                                                                      \
            heap[i].key   = key;                                                      \
            heap[i].val   = node;                                                     \
            locator[node] = i;                                                        \
        }                                                                             \
                                                                                      \
        ASSERT2(FPRFX##CheckHeap(queue));                                             \
        return vtx;                                                                   \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function returns the item at the top of the queue. The item is not\
    deleted from the queue. */      \
    /**************************************************************************/      \
    VT FPRFX##SeeTopVal(PQT* queue)                                                   \
    {                                                                                 \
        return (queue->nnodes == 0 ? -1 : queue->heap[0].val);                        \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function returns the key of the top item. The item is not\
    deleted from the queue. */               \
    /**************************************************************************/      \
    KT FPRFX##SeeTopKey(PQT* queue)                                                   \
    {                                                                                 \
        return (queue->nnodes == 0 ? KMAX : queue->heap[0].key);                      \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function returns the key of a specific item */                           \
    /**************************************************************************/      \
    KT FPRFX##SeeKey(PQT* queue, VT node)                                             \
    {                                                                                 \
        ssize_t* locator;                                                             \
        KVT*     heap;                                                                \
                                                                                      \
        heap    = queue->heap;                                                        \
        locator = queue->locator;                                                     \
                                                                                      \
        return heap[locator[node]].key;                                               \
    }                                                                                 \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This function returns the first item in a breadth-first traversal of\
    the heap whose key is less than maxwgt. This function is here due to\
    hMETIS and is not general!*/         \
    /**************************************************************************/      \
    /*\
VT FPRFX ## SeeConstraintTop(PQT *queue, KT maxwgt, KT *wgts)\
{\
  ssize_t i;\
\
  if (queue->nnodes == 0)\
    return -1;\
\
  if (maxwgt <= 1000)\
    return FPRFX ## SeeTopVal(queue);\
\
  for (i=0; i<queue->nnodes; i++) {\
    if (queue->heap[i].key > 0) {\
      if (wgts[queue->heap[i].val] <= maxwgt)\
        return queue->heap[i].val;\
    }\
    else {\
      if (queue->heap[i/2].key <= 0)\
        break;\
    }\
  }\
\
  return queue->heap[0].val;\
\
}\
*/                                                                               \
                                                                                      \
                                                                                      \
    /*************************************************************************/       \
    /*! This functions checks the consistency of the heap */                          \
    /**************************************************************************/      \
    int FPRFX##CheckHeap(PQT* queue)                                                  \
    {                                                                                 \
        ssize_t  i, j;                                                                \
        size_t   nnodes;                                                              \
        ssize_t* locator;                                                             \
        KVT*     heap;                                                                \
                                                                                      \
        heap    = queue->heap;                                                        \
        locator = queue->locator;                                                     \
        nnodes  = queue->nnodes;                                                      \
                                                                                      \
        if(nnodes == 0)                                                               \
            return 1;                                                                 \
                                                                                      \
        ASSERT(locator[heap[0].val] == 0);                                            \
        for(i = 1; i < nnodes; i++)                                                   \
        {                                                                             \
            ASSERT(locator[heap[i].val] == i);                                        \
            ASSERT(!KEY_LT(heap[i].key, heap[(i - 1) / 2].key));                      \
        }                                                                             \
        for(i = 1; i < nnodes; i++)                                                   \
            ASSERT(!KEY_LT(heap[i].key, heap[0].key));                                \
                                                                                      \
        for(j = i = 0; i < queue->maxnodes; i++)                                      \
        {                                                                             \
            if(locator[i] != -1)                                                      \
                j++;                                                                  \
        }                                                                             \
        ASSERTP(j == nnodes, ("%jd %jd\n", (intmax_t)j, (intmax_t)nnodes));           \
                                                                                      \
        return 1;                                                                     \
    }


#define GK_MKPQUEUE_PROTO(FPRFX, PQT, KT, VT)                                  \
    PQT*   FPRFX##Create(size_t maxnodes);                                     \
    void   FPRFX##Init(PQT* queue, size_t maxnodes);                           \
    void   FPRFX##Reset(PQT* queue);                                           \
    void   FPRFX##Free(PQT* queue);                                            \
    void   FPRFX##Destroy(PQT* queue);                                         \
    size_t FPRFX##Length(PQT* queue);                                          \
    int    FPRFX##Insert(PQT* queue, VT node, KT key);                         \
    int    FPRFX##Delete(PQT* queue, VT node);                                 \
    void   FPRFX##Update(PQT* queue, VT node, KT newkey);                      \
    VT     FPRFX##GetTop(PQT* queue);                                          \
    VT     FPRFX##SeeTopVal(PQT* queue);                                       \
    KT     FPRFX##SeeTopKey(PQT* queue);                                       \
    KT     FPRFX##SeeKey(PQT* queue, VT node);                                 \
    VT     FPRFX##SeeConstraintTop(PQT* queue, KT maxwgt, KT* wgts);           \
    int    FPRFX##CheckHeap(PQT* queue);


/* This is how these macros are used
GK_MKPQUEUE(gk_dkvPQ, gk_dkvPQ_t, double, gk_idx_t, gk_dkvmalloc, DBL_MAX)
GK_MKPQUEUE_PROTO(gk_dkvPQ, gk_dkvPQ_t, double, gk_idx_t)
*/


#endif

/*!
\file  gk_mkpqueue2.h
\brief Templates for priority queues that do not utilize locators and as such
       they can use different types of values.

\date   Started 4/09/07
\author George
\version\verbatim $Id: gk_mkpqueue2.h 13005 2012-10-23 22:34:36Z karypis $ \endverbatim
*/


#ifndef _GK_MKPQUEUE2_H
#define _GK_MKPQUEUE2_H


#define GK_MKPQUEUE2(FPRFX, PQT, KT, VT, KMALLOC, VMALLOC, KMAX, KEY_LT)          \
    /*************************************************************************/   \
    /*! This function creates and initializes a priority queue */                 \
    /**************************************************************************/  \
    PQT* FPRFX##Create2(ssize_t maxnodes)                                         \
    {                                                                             \
        PQT* queue;                                                               \
                                                                                  \
        if((queue = (PQT*)gk_malloc(sizeof(PQT), "gk_pqCreate2: queue")) != NULL) \
        {                                                                         \
            memset(queue, 0, sizeof(PQT));                                        \
            queue->nnodes   = 0;                                                  \
            queue->maxnodes = maxnodes;                                           \
            queue->keys     = KMALLOC(maxnodes, "gk_pqCreate2: keys");            \
            queue->vals     = VMALLOC(maxnodes, "gk_pqCreate2: vals");            \
                                                                                  \
            if(queue->keys == NULL || queue->vals == NULL)                        \
                gk_free((void**)&queue->keys, &queue->vals, &queue, LTERM);       \
        }                                                                         \
                                                                                  \
        return queue;                                                             \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function resets the priority queue */                                \
    /**************************************************************************/  \
    void FPRFX##Reset2(PQT* queue)                                                \
    {                                                                             \
        queue->nnodes = 0;                                                        \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function frees the internal datastructures of the priority queue */  \
    /**************************************************************************/  \
    void FPRFX##Destroy2(PQT** r_queue)                                           \
    {                                                                             \
        PQT* queue = *r_queue;                                                    \
        if(queue == NULL)                                                         \
            return;                                                               \
        gk_free((void**)&queue->keys, &queue->vals, &queue, LTERM);               \
        *r_queue = NULL;                                                          \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function returns the length of the queue */                          \
    /**************************************************************************/  \
    size_t FPRFX##Length2(PQT* queue)                                             \
    {                                                                             \
        return queue->nnodes;                                                     \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function adds an item in the priority queue. */                      \
    /**************************************************************************/  \
    int FPRFX##Insert2(PQT* queue, VT val, KT key)                                \
    {                                                                             \
        ssize_t i, j;                                                             \
        KT*     keys = queue->keys;                                               \
        VT*     vals = queue->vals;                                               \
                                                                                  \
        ASSERT2(FPRFX##CheckHeap2(queue));                                        \
                                                                                  \
        if(queue->nnodes == queue->maxnodes)                                      \
            return 0;                                                             \
                                                                                  \
        ASSERT2(FPRFX##CheckHeap2(queue));                                        \
                                                                                  \
        i = queue->nnodes++;                                                      \
        while(i > 0)                                                              \
        {                                                                         \
            j = (i - 1) >> 1;                                                     \
            if(KEY_LT(key, keys[j]))                                              \
            {                                                                     \
                keys[i] = keys[j];                                                \
                vals[i] = vals[j];                                                \
                i       = j;                                                      \
            }                                                                     \
            else                                                                  \
                break;                                                            \
        }                                                                         \
        ASSERT(i >= 0);                                                           \
        keys[i] = key;                                                            \
        vals[i] = val;                                                            \
                                                                                  \
        ASSERT2(FPRFX##CheckHeap2(queue));                                        \
                                                                                  \
        return 1;                                                                 \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function returns the item at the top of the queue and removes\
    it from the priority queue */       \
    /**************************************************************************/  \
    int FPRFX##GetTop2(PQT* queue, VT* r_val)                                     \
    {                                                                             \
        ssize_t i, j;                                                             \
        KT      key, *keys = queue->keys;                                         \
        VT      val, *vals = queue->vals;                                         \
                                                                                  \
        ASSERT2(FPRFX##CheckHeap2(queue));                                        \
                                                                                  \
        if(queue->nnodes == 0)                                                    \
            return 0;                                                             \
                                                                                  \
        queue->nnodes--;                                                          \
                                                                                  \
        *r_val = vals[0];                                                         \
                                                                                  \
        if((i = queue->nnodes) > 0)                                               \
        {                                                                         \
            key = keys[i];                                                        \
            val = vals[i];                                                        \
            i   = 0;                                                              \
            while((j = 2 * i + 1) < queue->nnodes)                                \
            {                                                                     \
                if(KEY_LT(keys[j], key))                                          \
                {                                                                 \
                    if(j + 1 < queue->nnodes && KEY_LT(keys[j + 1], keys[j]))     \
                        j = j + 1;                                                \
                    keys[i] = keys[j];                                            \
                    vals[i] = vals[j];                                            \
                    i       = j;                                                  \
                }                                                                 \
                else if(j + 1 < queue->nnodes && KEY_LT(keys[j + 1], key))        \
                {                                                                 \
                    j       = j + 1;                                              \
                    keys[i] = keys[j];                                            \
                    vals[i] = vals[j];                                            \
                    i       = j;                                                  \
                }                                                                 \
                else                                                              \
                    break;                                                        \
            }                                                                     \
                                                                                  \
            keys[i] = key;                                                        \
            vals[i] = val;                                                        \
        }                                                                         \
                                                                                  \
        ASSERT2(FPRFX##CheckHeap2(queue));                                        \
                                                                                  \
        return 1;                                                                 \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function returns the item at the top of the queue. The item is not\
    deleted from the queue. */  \
    /**************************************************************************/  \
    int FPRFX##SeeTopVal2(PQT* queue, VT* r_val)                                  \
    {                                                                             \
        if(queue->nnodes == 0)                                                    \
            return 0;                                                             \
                                                                                  \
        *r_val = queue->vals[0];                                                  \
                                                                                  \
        return 1;                                                                 \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This function returns the key of the top item. The item is not\
    deleted from the queue. */           \
    /**************************************************************************/  \
    KT FPRFX##SeeTopKey2(PQT* queue)                                              \
    {                                                                             \
        return (queue->nnodes == 0 ? KMAX : queue->keys[0]);                      \
    }                                                                             \
                                                                                  \
                                                                                  \
    /*************************************************************************/   \
    /*! This functions checks the consistency of the heap */                      \
    /**************************************************************************/  \
    int FPRFX##CheckHeap2(PQT* queue)                                             \
    {                                                                             \
        ssize_t i;                                                                \
        KT*     keys = queue->keys;                                               \
                                                                                  \
        if(queue->nnodes == 0)                                                    \
            return 1;                                                             \
                                                                                  \
        for(i = 1; i < queue->nnodes; i++)                                        \
        {                                                                         \
            ASSERT(!KEY_LT(keys[i], keys[(i - 1) / 2]));                          \
        }                                                                         \
        for(i = 1; i < queue->nnodes; i++)                                        \
            ASSERT(!KEY_LT(keys[i], keys[0]));                                    \
                                                                                  \
        return 1;                                                                 \
    }


#define GK_MKPQUEUE2_PROTO(FPRFX, PQT, KT, VT)                                 \
    PQT*   FPRFX##Create2(ssize_t maxnodes);                                   \
    void   FPRFX##Reset2(PQT* queue);                                          \
    void   FPRFX##Destroy2(PQT** r_queue);                                     \
    size_t FPRFX##Length2(PQT* queue);                                         \
    int    FPRFX##Insert2(PQT* queue, VT node, KT key);                        \
    int    FPRFX##GetTop2(PQT* queue, VT* r_val);                              \
    int    FPRFX##SeeTopVal2(PQT* queue, VT* r_val);                           \
    KT     FPRFX##SeeTopKey2(PQT* queue);                                      \
    int    FPRFX##CheckHeap2(PQT* queue);


#endif

/*!
\file
\brief Templates for portable random number generation

\date   Started 5/17/07
\author George
\version\verbatim $Id: gk_mkrandom.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#ifndef _GK_MKRANDOM_H
#define _GK_MKRANDOM_H

/*************************************************************************/\
/*! The generator for the rand() related routines.  \
   \params RNGT  the datatype that defines the range of values over which\
                 random numbers will be generated\
   \params VALT  the datatype that defines the contents of the array to \
                 be permuted by randArrayPermute() \
   \params FPRFX the function prefix \
*/\
/**************************************************************************/\
#define GK_MKRANDOM(FPRFX, RNGT, VALT)     \
    /*************************************************************************/  \
    /*! Initializes the generator */                                             \
    /**************************************************************************/ \
    void FPRFX##srand(RNGT seed)                                                 \
    {                                                                            \
        gk_randinit((uint64_t)seed);                                             \
    }                                                                            \
                                                                                 \
                                                                                 \
    /*************************************************************************/  \
    /*! Returns a random number */                                               \
    /**************************************************************************/ \
    RNGT FPRFX##rand()                                                           \
    {                                                                            \
        if(sizeof(RNGT) <= sizeof(int32_t))                                      \
            return (RNGT)gk_randint32();                                         \
        else                                                                     \
            return (RNGT)gk_randint64();                                         \
    }                                                                            \
                                                                                 \
                                                                                 \
    /*************************************************************************/  \
    /*! Returns a random number between [0, max) */                              \
    /**************************************************************************/ \
    RNGT FPRFX##randInRange(RNGT max)                                            \
    {                                                                            \
        return (RNGT)((FPRFX##rand()) % max);                                    \
    }                                                                            \
                                                                                 \
                                                                                 \
    /*************************************************************************/  \
    /*! Randomly permutes the elements of an array p[]. \
    flag == 1, p[i] = i prior to permutation, \
    flag == 0, p[] is not initialized. */                        \
    /**************************************************************************/ \
    void FPRFX##randArrayPermute(RNGT n, VALT* p, RNGT nshuffles, int flag)      \
    {                                                                            \
        RNGT i, u, v;                                                            \
        VALT tmp;                                                                \
                                                                                 \
        if(flag == 1)                                                            \
        {                                                                        \
            for(i = 0; i < n; i++)                                               \
                p[i] = (VALT)i;                                                  \
        }                                                                        \
                                                                                 \
        if(n < 10)                                                               \
        {                                                                        \
            for(i = 0; i < n; i++)                                               \
            {                                                                    \
                v = FPRFX##randInRange(n);                                       \
                u = FPRFX##randInRange(n);                                       \
                gk_SWAP(p[v], p[u], tmp);                                        \
            }                                                                    \
        }                                                                        \
        else                                                                     \
        {                                                                        \
            for(i = 0; i < nshuffles; i++)                                       \
            {                                                                    \
                v = FPRFX##randInRange(n - 3);                                   \
                u = FPRFX##randInRange(n - 3);                                   \
                /*gk_SWAP(p[v+0], p[u+0], tmp);*/                                \
                /*gk_SWAP(p[v+1], p[u+1], tmp);*/                                \
                /*gk_SWAP(p[v+2], p[u+2], tmp);*/                                \
                /*gk_SWAP(p[v+3], p[u+3], tmp);*/                                \
                gk_SWAP(p[v + 0], p[u + 2], tmp);                                \
                gk_SWAP(p[v + 1], p[u + 3], tmp);                                \
                gk_SWAP(p[v + 2], p[u + 0], tmp);                                \
                gk_SWAP(p[v + 3], p[u + 1], tmp);                                \
            }                                                                    \
        }                                                                        \
    }                                                                            \
                                                                                 \
                                                                                 \
    /*************************************************************************/  \
    /*! Randomly permutes the elements of an array p[]. \
    flag == 1, p[i] = i prior to permutation, \
    flag == 0, p[] is not initialized. */                        \
    /**************************************************************************/ \
    void FPRFX##randArrayPermuteFine(RNGT n, VALT* p, int flag)                  \
    {                                                                            \
        RNGT i, v;                                                               \
        VALT tmp;                                                                \
                                                                                 \
        if(flag == 1)                                                            \
        {                                                                        \
            for(i = 0; i < n; i++)                                               \
                p[i] = (VALT)i;                                                  \
        }                                                                        \
                                                                                 \
        for(i = 0; i < n; i++)                                                   \
        {                                                                        \
            v = FPRFX##randInRange(n);                                           \
            gk_SWAP(p[i], p[v], tmp);                                            \
        }                                                                        \
    }


#define GK_MKRANDOM_PROTO(FPRFX, RNGT, VALT)                                   \
    void FPRFX##srand(RNGT seed);                                              \
    RNGT FPRFX##rand();                                                        \
    RNGT FPRFX##randInRange(RNGT max);                                         \
    void FPRFX##randArrayPermute(RNGT n, VALT* p, RNGT nshuffles, int flag);   \
    void FPRFX##randArrayPermuteFine(RNGT n, VALT* p, int flag);


#endif

/*!
\file
\brief Templates for various utility routines

\date   Started 5/28/07
\author George
\version\verbatim $Id: gk_mkutils.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_MKUTILS_H_
#define _GK_MKUTILS_H_


#define GK_MKARRAY2CSR(PRFX, TYPE)                                               \
    /*************************************************************************/  \
    /*! The macro for gk_?array2csr() routine */                                 \
    /**************************************************************************/ \
    void PRFX##array2csr(TYPE n, TYPE range, TYPE* array, TYPE* ptr, TYPE* ind)  \
    {                                                                            \
        TYPE i;                                                                  \
                                                                                 \
        for(i = 0; i <= range; i++)                                              \
            ptr[i] = 0;                                                          \
                                                                                 \
        for(i = 0; i < n; i++)                                                   \
            ptr[array[i]]++;                                                     \
                                                                                 \
        /* Compute the ptr, ind structure */                                     \
        MAKECSR(i, range, ptr);                                                  \
        for(i = 0; i < n; i++)                                                   \
            ind[ptr[array[i]]++] = i;                                            \
        SHIFTCSR(i, range, ptr);                                                 \
    }


#define GK_MKARRAY2CSR_PROTO(PRFX, TYPE)                                       \
    void PRFX##array2csr(TYPE n, TYPE range, TYPE* array, TYPE* ptr, TYPE* ind);


#endif

/*!
\file gk_proto.h
\brief This file contains function prototypes

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_proto.h 22010 2018-05-14 20:20:26Z karypis $ \endverbatim
*/

#ifndef _GK_PROTO_H_
#define _GK_PROTO_H_

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------------------------------------------
 * blas.c
 *-------------------------------------------------------------*/
GK_MKBLAS_PROTO(gk_c, char, int)
GK_MKBLAS_PROTO(gk_i, int, int)
GK_MKBLAS_PROTO(gk_i8, int8_t, int8_t)
GK_MKBLAS_PROTO(gk_i16, int16_t, int16_t)
GK_MKBLAS_PROTO(gk_i32, int32_t, int32_t)
GK_MKBLAS_PROTO(gk_i64, int64_t, int64_t)
GK_MKBLAS_PROTO(gk_z, ssize_t, ssize_t)
GK_MKBLAS_PROTO(gk_zu, size_t, size_t)
GK_MKBLAS_PROTO(gk_f, float, float)
GK_MKBLAS_PROTO(gk_d, double, double)
GK_MKBLAS_PROTO(gk_idx, gk_idx_t, gk_idx_t)


/*-------------------------------------------------------------
 * io.c
 *-------------------------------------------------------------*/
FILE*    gk_fopen(char*, char*, const char*);
void     gk_fclose(FILE*);
ssize_t  gk_read(int fd, void* vbuf, size_t count);
ssize_t  gk_write(int fd, void* vbuf, size_t count);
ssize_t  gk_getline(char** lineptr, size_t* n, FILE* stream);
char**   gk_readfile(char* fname, size_t* r_nlines);
int32_t* gk_i32readfile(char* fname, size_t* r_nlines);
int64_t* gk_i64readfile(char* fname, size_t* r_nlines);
ssize_t* gk_zreadfile(char* fname, size_t* r_nlines);
char*    gk_creadfilebin(char* fname, size_t* r_nelmnts);
size_t   gk_cwritefilebin(char* fname, size_t n, char* a);
int32_t* gk_i32readfilebin(char* fname, size_t* r_nelmnts);
size_t   gk_i32writefilebin(char* fname, size_t n, int32_t* a);
int64_t* gk_i64readfilebin(char* fname, size_t* r_nelmnts);
size_t   gk_i64writefilebin(char* fname, size_t n, int64_t* a);
ssize_t* gk_zreadfilebin(char* fname, size_t* r_nelmnts);
size_t   gk_zwritefilebin(char* fname, size_t n, ssize_t* a);
float*   gk_freadfilebin(char* fname, size_t* r_nelmnts);
size_t   gk_fwritefilebin(char* fname, size_t n, float* a);
double*  gk_dreadfilebin(char* fname, size_t* r_nelmnts);
size_t   gk_dwritefilebin(char* fname, size_t n, double* a);


/*-------------------------------------------------------------
 * fs.c
 *-------------------------------------------------------------*/
int     gk_fexists(char*);
int     gk_dexists(char*);
ssize_t gk_getfsize(char*);
void gk_getfilestats(char* fname, size_t* r_nlines, size_t* r_ntokens, size_t* r_max_nlntokens, size_t* r_nbytes);
char* gk_getbasename(char* path);
char* gk_getextname(char* path);
char* gk_getfilename(char* path);
char* gk_getpathname(char* path);
int   gk_mkpath(char*);
int   gk_rmpath(char*);


/*-------------------------------------------------------------
 * memory.c
 *-------------------------------------------------------------*/
GK_MKALLOC_PROTO(gk_c, char)
GK_MKALLOC_PROTO(gk_i, int)
GK_MKALLOC_PROTO(gk_i8, int8_t)
GK_MKALLOC_PROTO(gk_i16, int16_t)
GK_MKALLOC_PROTO(gk_i32, int32_t)
GK_MKALLOC_PROTO(gk_i64, int64_t)
GK_MKALLOC_PROTO(gk_ui8, uint8_t)
GK_MKALLOC_PROTO(gk_ui16, uint16_t)
GK_MKALLOC_PROTO(gk_ui32, uint32_t)
GK_MKALLOC_PROTO(gk_ui64, uint64_t)
GK_MKALLOC_PROTO(gk_z, ssize_t)
GK_MKALLOC_PROTO(gk_zu, size_t)
GK_MKALLOC_PROTO(gk_f, float)
GK_MKALLOC_PROTO(gk_d, double)
GK_MKALLOC_PROTO(gk_idx, gk_idx_t)

GK_MKALLOC_PROTO(gk_ckv, gk_ckv_t)
GK_MKALLOC_PROTO(gk_ikv, gk_ikv_t)
GK_MKALLOC_PROTO(gk_i8kv, gk_i8kv_t)
GK_MKALLOC_PROTO(gk_i16kv, gk_i16kv_t)
GK_MKALLOC_PROTO(gk_i32kv, gk_i32kv_t)
GK_MKALLOC_PROTO(gk_i64kv, gk_i64kv_t)
GK_MKALLOC_PROTO(gk_zkv, gk_zkv_t)
GK_MKALLOC_PROTO(gk_zukv, gk_zukv_t)
GK_MKALLOC_PROTO(gk_fkv, gk_fkv_t)
GK_MKALLOC_PROTO(gk_dkv, gk_dkv_t)
GK_MKALLOC_PROTO(gk_skv, gk_skv_t)
GK_MKALLOC_PROTO(gk_idxkv, gk_idxkv_t)

void   gk_AllocMatrix(void***, size_t, size_t, size_t);
void   gk_FreeMatrix(void***, size_t, size_t);
int    gk_malloc_init();
void   gk_malloc_cleanup(int showstats);
void*  gk_malloc(size_t nbytes, const char* msg);
void*  gk_realloc(void* oldptr, size_t nbytes, const char* msg);
void   gk_free(void** ptr1, ...);
size_t gk_GetCurMemoryUsed();
size_t gk_GetMaxMemoryUsed();


/*-------------------------------------------------------------
 * seq.c
 *-------------------------------------------------------------*/
gk_seq_t*    gk_seq_ReadGKMODPSSM(char* file_name);
gk_i2cc2i_t* gk_i2cc2i_create_common(char* alphabet);
void         gk_seq_init(gk_seq_t* seq);


/*-------------------------------------------------------------
 * error.c
 *-------------------------------------------------------------*/
void  gk_set_exit_on_error(int value);
void  errexit(const char*, ...);
void  gk_errexit(int signum, const char*, ...);
int   gk_sigtrap();
int   gk_siguntrap();
void  gk_sigthrow(int signum);
void  gk_SetSignalHandlers();
void  gk_UnsetSignalHandlers();
void  gk_NonLocalExit_Handler(int signum);
char* gk_strerror(int errnum);
void  PrintBackTrace();


/*-------------------------------------------------------------
 * util.c
 *-------------------------------------------------------------*/
void  gk_RandomPermute(size_t, int*, int);
void  gk_array2csr(size_t n, size_t range, int* array, int* ptr, int* ind);
int   gk_log2(int);
int   gk_ispow2(int);
float gk_flog2(float);


/*-------------------------------------------------------------
 * time.c
 *-------------------------------------------------------------*/
gk_wclock_t gk_WClockSeconds(void);
double      gk_CPUSeconds(void);

/*-------------------------------------------------------------
 * string.c
 *-------------------------------------------------------------*/
char* gk_strchr_replace(char* str, char* fromlist, char* tolist);
int gk_strstr_replace(char* str, char* pattern, char* replacement, char* options, char** new_str);
char*  gk_strtprune(char*, char*);
char*  gk_strhprune(char*, char*);
char*  gk_strtoupper(char*);
char*  gk_strtolower(char*);
char*  gk_strdup(char* orgstr);
int    gk_strcasecmp(char* s1, char* s2);
int    gk_strrcmp(char* s1, char* s2);
char*  gk_time2str(time_t time);
time_t gk_str2time(char* str);
int    gk_GetStringID(gk_StringMap_t* strmap, char* key);


/*-------------------------------------------------------------
 * sort.c
 *-------------------------------------------------------------*/
void gk_csorti(size_t, char*);
void gk_csortd(size_t, char*);
void gk_isorti(size_t, int*);
void gk_isortd(size_t, int*);
void gk_i32sorti(size_t, int32_t*);
void gk_i32sortd(size_t, int32_t*);
void gk_i64sorti(size_t, int64_t*);
void gk_i64sortd(size_t, int64_t*);
void gk_ui32sorti(size_t, uint32_t*);
void gk_ui32sortd(size_t, uint32_t*);
void gk_ui64sorti(size_t, uint64_t*);
void gk_ui64sortd(size_t, uint64_t*);
void gk_fsorti(size_t, float*);
void gk_fsortd(size_t, float*);
void gk_dsorti(size_t, double*);
void gk_dsortd(size_t, double*);
void gk_idxsorti(size_t, gk_idx_t*);
void gk_idxsortd(size_t, gk_idx_t*);
void gk_ckvsorti(size_t, gk_ckv_t*);
void gk_ckvsortd(size_t, gk_ckv_t*);
void gk_ikvsorti(size_t, gk_ikv_t*);
void gk_ikvsortd(size_t, gk_ikv_t*);
void gk_i32kvsorti(size_t, gk_i32kv_t*);
void gk_i32kvsortd(size_t, gk_i32kv_t*);
void gk_i64kvsorti(size_t, gk_i64kv_t*);
void gk_i64kvsortd(size_t, gk_i64kv_t*);
void gk_zkvsorti(size_t, gk_zkv_t*);
void gk_zkvsortd(size_t, gk_zkv_t*);
void gk_zukvsorti(size_t, gk_zukv_t*);
void gk_zukvsortd(size_t, gk_zukv_t*);
void gk_fkvsorti(size_t, gk_fkv_t*);
void gk_fkvsortd(size_t, gk_fkv_t*);
void gk_dkvsorti(size_t, gk_dkv_t*);
void gk_dkvsortd(size_t, gk_dkv_t*);
void gk_skvsorti(size_t, gk_skv_t*);
void gk_skvsortd(size_t, gk_skv_t*);
void gk_idxkvsorti(size_t, gk_idxkv_t*);
void gk_idxkvsortd(size_t, gk_idxkv_t*);


/*-------------------------------------------------------------
 * Selection routines
 *-------------------------------------------------------------*/
int gk_dfkvkselect(size_t, int, gk_fkv_t*);
int gk_ifkvkselect(size_t, int, gk_fkv_t*);


/*-------------------------------------------------------------
 * Priority queue
 *-------------------------------------------------------------*/
GK_MKPQUEUE_PROTO(gk_ipq, gk_ipq_t, int, gk_idx_t)
GK_MKPQUEUE_PROTO(gk_i32pq, gk_i32pq_t, int32_t, gk_idx_t)
GK_MKPQUEUE_PROTO(gk_i64pq, gk_i64pq_t, int64_t, gk_idx_t)
GK_MKPQUEUE_PROTO(gk_fpq, gk_fpq_t, float, gk_idx_t)
GK_MKPQUEUE_PROTO(gk_dpq, gk_dpq_t, double, gk_idx_t)
GK_MKPQUEUE_PROTO(gk_idxpq, gk_idxpq_t, gk_idx_t, gk_idx_t)


/*-------------------------------------------------------------
 * HTable routines
 *-------------------------------------------------------------*/
gk_HTable_t* HTable_Create(int nelements);
void         HTable_Reset(gk_HTable_t* htable);
void         HTable_Resize(gk_HTable_t* htable, int nelements);
void         HTable_Insert(gk_HTable_t* htable, int key, int val);
void         HTable_Delete(gk_HTable_t* htable, int key);
int          HTable_Search(gk_HTable_t* htable, int key);
int          HTable_GetNext(gk_HTable_t* htable, int key, int* val, int type);
int          HTable_SearchAndDelete(gk_HTable_t* htable, int key);
void         HTable_Destroy(gk_HTable_t* htable);
int          HTable_HFunction(int nelements, int key);


/*-------------------------------------------------------------
 * Tokenizer routines
 *-------------------------------------------------------------*/
void gk_strtokenize(char* line, char* delim, gk_Tokens_t* tokens);
void gk_freetokenslist(gk_Tokens_t* tokens);

/*-------------------------------------------------------------
 * Encoder/Decoder
 *-------------------------------------------------------------*/
void encodeblock(unsigned char* in, unsigned char* out);
void decodeblock(unsigned char* in, unsigned char* out);
void GKEncodeBase64(int nbytes, unsigned char* inbuffer, unsigned char* outbuffer);
void GKDecodeBase64(int nbytes, unsigned char* inbuffer, unsigned char* outbuffer);


/*-------------------------------------------------------------
 * random.c
 *-------------------------------------------------------------*/
GK_MKRANDOM_PROTO(gk_c, size_t, char)
GK_MKRANDOM_PROTO(gk_i, size_t, int)
GK_MKRANDOM_PROTO(gk_i32, size_t, int32_t)
GK_MKRANDOM_PROTO(gk_f, size_t, float)
GK_MKRANDOM_PROTO(gk_d, size_t, double)
GK_MKRANDOM_PROTO(gk_idx, size_t, gk_idx_t)
GK_MKRANDOM_PROTO(gk_z, size_t, ssize_t)
GK_MKRANDOM_PROTO(gk_zu, size_t, size_t)
void     gk_randinit(uint64_t);
uint64_t gk_randint64(void);
uint32_t gk_randint32(void);


/*-------------------------------------------------------------
 * OpenMP fake functions
 *-------------------------------------------------------------*/
#if !defined(__OPENMP__)
void omp_set_num_threads(int num_threads);
int  omp_get_num_threads(void);
int  omp_get_max_threads(void);
int  omp_get_thread_num(void);
int  omp_get_num_procs(void);
int  omp_in_parallel(void);
void omp_set_dynamic(int num_threads);
int  omp_get_dynamic(void);
void omp_set_nested(int nested);
int  omp_get_nested(void);
#endif /* __OPENMP__ */


/*-------------------------------------------------------------
 * CSR-related functions
 *-------------------------------------------------------------*/
gk_csr_t*  gk_csr_Create();
void       gk_csr_Init(gk_csr_t* mat);
void       gk_csr_Free(gk_csr_t** mat);
void       gk_csr_FreeContents(gk_csr_t* mat);
gk_csr_t*  gk_csr_Dup(gk_csr_t* mat);
gk_csr_t*  gk_csr_ExtractSubmatrix(gk_csr_t* mat, int rstart, int nrows);
gk_csr_t*  gk_csr_ExtractRows(gk_csr_t* mat, int nrows, int* rind);
gk_csr_t*  gk_csr_ExtractPartition(gk_csr_t* mat, int* part, int pid);
gk_csr_t** gk_csr_Split(gk_csr_t* mat, int* color);
int        gk_csr_DetermineFormat(char* filename, int format);
gk_csr_t*  gk_csr_Read(char* filename, int format, int readvals, int numbering);
void gk_csr_Write(gk_csr_t* mat, char* filename, int format, int writevals, int numbering);
gk_csr_t* gk_csr_Prune(gk_csr_t* mat, int what, int minf, int maxf);
gk_csr_t* gk_csr_LowFilter(gk_csr_t* mat, int what, int norm, float fraction);
gk_csr_t* gk_csr_TopKPlusFilter(gk_csr_t* mat, int what, int topk, float keepval);
gk_csr_t* gk_csr_ZScoreFilter(gk_csr_t* mat, int what, float zscore);
void      gk_csr_CompactColumns(gk_csr_t* mat);
void      gk_csr_SortIndices(gk_csr_t* mat, int what);
void      gk_csr_CreateIndex(gk_csr_t* mat, int what);
void      gk_csr_Normalize(gk_csr_t* mat, int what, int norm);
void      gk_csr_Scale(gk_csr_t* mat, int type);
void      gk_csr_ComputeSums(gk_csr_t* mat, int what);
void      gk_csr_ComputeNorms(gk_csr_t* mat, int what);
void      gk_csr_ComputeSquaredNorms(gk_csr_t* mat, int what);
gk_csr_t* gk_csr_Shuffle(gk_csr_t* mat, int what, int summetric);
gk_csr_t* gk_csr_Transpose(gk_csr_t* mat);
float gk_csr_ComputeSimilarity(gk_csr_t* mat, int i1, int i2, int what, int simtype);
float gk_csr_ComputePairSimilarity(gk_csr_t* mat_a, gk_csr_t* mat_b, int i1, int i2, int what, int simtype);
int gk_csr_GetSimilarRows(gk_csr_t* mat,
                          int       nqterms,
                          int*      qind,
                          float*    qval,
                          int       simtype,
                          int       nsim,
                          float     minsim,
                          gk_fkv_t* hits,
                          int*      _imarker,
                          gk_fkv_t* i_cand);
int gk_csr_FindConnectedComponents(gk_csr_t* mat, int32_t* cptr, int32_t* cind, int32_t* cids);
gk_csr_t* gk_csr_MakeSymmetric(gk_csr_t* mat, int op);
gk_csr_t* gk_csr_ReorderSymmetric(gk_csr_t* mat, int32_t* perm, int32_t* iperm);
void      gk_csr_ComputeBFSOrderingSymmetric(
         gk_csr_t* mat, int maxdegree, int v, int32_t** r_perm, int32_t** r_iperm);
void gk_csr_ComputeBestFOrderingSymmetric(
    gk_csr_t* mat, int v, int type, int32_t** r_perm, int32_t** r_iperm);


/* itemsets.c */
void gk_find_frequent_itemsets(
    int      ntrans,
    ssize_t* tranptr,
    int*     tranind,
    int      minfreq,
    int      maxfreq,
    int      minlen,
    int      maxlen,
    void (*process_itemset)(void* stateptr, int nitems, int* itemind, int ntrans, int* tranind),
    void* stateptr);


/* evaluate.c */
float ComputeAccuracy(int n, gk_fkv_t* list);
float ComputeROCn(int n, int maxN, gk_fkv_t* list);
float ComputeMedianRFP(int n, gk_fkv_t* list);
float ComputeMean(int n, float* values);
float ComputeStdDev(int n, float* values);


/* mcore.c */
gk_mcore_t* gk_mcoreCreate(size_t coresize);
gk_mcore_t* gk_gkmcoreCreate();
void        gk_mcoreDestroy(gk_mcore_t** r_mcore, int showstats);
void        gk_gkmcoreDestroy(gk_mcore_t** r_mcore, int showstats);
void*       gk_mcoreMalloc(gk_mcore_t* mcore, size_t nbytes);
void        gk_mcorePush(gk_mcore_t* mcore);
void        gk_gkmcorePush(gk_mcore_t* mcore);
void        gk_mcorePop(gk_mcore_t* mcore);
void        gk_gkmcorePop(gk_mcore_t* mcore);
void        gk_mcoreAdd(gk_mcore_t* mcore, int type, size_t nbytes, void* ptr);
void gk_gkmcoreAdd(gk_mcore_t* mcore, int type, size_t nbytes, void* ptr);
void gk_mcoreDel(gk_mcore_t* mcore, void* ptr);
void gk_gkmcoreDel(gk_mcore_t* mcore, void* ptr);

/* rw.c */
int gk_rw_PageRank(gk_csr_t* mat, float lamda, float eps, int max_niter, float* pr);


/* graph.c */
gk_graph_t* gk_graph_Create();
void        gk_graph_Init(gk_graph_t* graph);
void        gk_graph_Free(gk_graph_t** graph);
void        gk_graph_FreeContents(gk_graph_t* graph);
gk_graph_t* gk_graph_Read(
    char* filename, int format, int hasvals, int numbering, int isfewgts, int isfvwgts, int isfvsizes);
void gk_graph_Write(gk_graph_t* graph, char* filename, int format, int numbering);
gk_graph_t* gk_graph_Dup(gk_graph_t* graph);
gk_graph_t* gk_graph_Transpose(gk_graph_t* graph);
gk_graph_t* gk_graph_ExtractSubgraph(gk_graph_t* graph, int vstart, int nvtxs);
gk_graph_t* gk_graph_Reorder(gk_graph_t* graph, int32_t* perm, int32_t* iperm);
int gk_graph_FindComponents(gk_graph_t* graph, int32_t* cptr, int32_t* cind);
void gk_graph_ComputeBFSOrdering(gk_graph_t* graph, int v, int32_t** r_perm, int32_t** r_iperm);
void gk_graph_ComputeBestFOrdering0(
    gk_graph_t* graph, int v, int type, int32_t** r_perm, int32_t** r_iperm);
void gk_graph_ComputeBestFOrdering(gk_graph_t* graph, int v, int type, int32_t** r_perm, int32_t** r_iperm);
void gk_graph_SingleSourceShortestPaths(gk_graph_t* graph, int v, void** r_sps);
void gk_graph_SortAdjacencies(gk_graph_t* graph);
gk_graph_t* gk_graph_MakeSymmetric(gk_graph_t* graph, int op);


/* cache.c */
gk_cache_t* gk_cacheCreate(uint32_t nway, uint32_t lnbits, size_t cnbits);
void        gk_cacheReset(gk_cache_t* cache);
void        gk_cacheDestroy(gk_cache_t** r_cache);
int         gk_cacheLoad(gk_cache_t* cache, size_t addr);
double      gk_cacheGetHitRate(gk_cache_t* cache);


#ifdef __cplusplus
}
#endif


#endif


#endif /* GKlib.h */
