/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 */

/*!
\file metis.h
\brief This file contains function prototypes and constant definitions for METIS
 *
\author George
\date   Started 8/9/02
\version\verbatim $Id$\endverbatim
*/

#ifndef _METIS_H_
#define _METIS_H_


#include <GKlib.h>
/****************************************************************************
* A set of defines that can be modified by the user
*****************************************************************************/

/*--------------------------------------------------------------------------
 Specifies the width of the elementary data type that will hold information
 about vertices and their adjacency lists.

 Possible values:
   32 : Use 32 bit signed integers
   64 : Use 64 bit signed integers

 A width of 64 should be specified if the number of vertices or the total
 number of edges in the graph exceed the limits of a 32 bit signed integer
 i.e., 2^31-1.
 Proper use of 64 bit integers requires that the c99 standard datatypes
 int32_t and int64_t are supported by the compiler.
 GCC does provides these definitions in stdint.h, but it may require some
 modifications on other architectures.
--------------------------------------------------------------------------*/
//#define IDXTYPEWIDTH 32


/*--------------------------------------------------------------------------
 Specifies the data type that will hold floating-point style information.

 Possible values:
   32 : single precision floating point (float)
   64 : double precision floating point (double)
--------------------------------------------------------------------------*/
//#define REALTYPEWIDTH 32


/****************************************************************************
* In principle, nothing needs to be changed beyond this point, unless the
* int32_t and int64_t cannot be found in the normal places.
*****************************************************************************/

/* Uniform definitions for various compilers */
#if defined(_MSC_VER)
#define COMPILER_MSC
#endif
#if defined(__ICC)
#define COMPILER_ICC
#endif
#if defined(__GNUC__)
#define COMPILER_GCC
#endif

/* Include c99 int definitions and need constants. When building the library,
 * these are already defined by GKlib; hence the test for _GKLIB_H_ */
#ifndef _GKLIB_H_
#ifdef COMPILER_MSC
#include <limits.h>

typedef __int32 int32_t;
typedef __int64 int64_t;
#define PRId32 "I32d"
#define PRId64 "I64d"
#define SCNd32 "ld"
#define SCNd64 "I64d"
#define INT32_MIN ((int32_t)_I32_MIN)
#define INT32_MAX _I32_MAX
#define INT64_MIN ((int64_t)_I64_MIN)
#define INT64_MAX _I64_MAX
#else
#include <inttypes.h>
#endif
#endif

#define REALTYPEWIDTH 64
#define IDXTYPEWIDTH 32
/*------------------------------------------------------------------------
* Setup the basic datatypes
*-------------------------------------------------------------------------*/
#if IDXTYPEWIDTH == 32
typedef int32_t idx_t;

#define IDX_MAX INT32_MAX
#define IDX_MIN INT32_MIN

#define SCIDX SCNd32
#define PRIDX PRId32

#define strtoidx strtol
#define iabs abs
#elif IDXTYPEWIDTH == 64
typedef int64_t idx_t;

#define IDX_MAX INT64_MAX
#define IDX_MIN INT64_MIN

#define SCIDX SCNd64
#define PRIDX PRId64

#ifdef COMPILER_MSC
#define strtoidx _strtoi64
#else
#define strtoidx strtoll
#endif
#define iabs labs
#else
#error "Incorrect user-supplied value of IDXTYPEWIDTH"
#endif


#if REALTYPEWIDTH == 32
typedef float real_t;

#define SCREAL "f"
#define PRREAL "f"
#define REAL_MAX FLT_MAX
#define REAL_MIN FLT_MIN
#define REAL_EPSILON FLT_EPSILON

#define rabs fabsf
#define REALEQ(x, y) ((rabs((x) - (y)) <= FLT_EPSILON))

#ifdef COMPILER_MSC
#define strtoreal (float)strtod
#else
#define strtoreal strtof
#endif
#elif REALTYPEWIDTH == 64
typedef double real_t;

#define SCREAL "lf"
#define PRREAL "lf"
#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN
#define REAL_EPSILON DBL_EPSILON

#define rabs fabs
#define REALEQ(x, y) ((rabs((x) - (y)) <= DBL_EPSILON))

#define strtoreal strtod
#else
#error "Incorrect user-supplied value for REALTYPEWIDTH"
#endif


/*------------------------------------------------------------------------
* Constant definitions
*-------------------------------------------------------------------------*/
/* Metis's version number */
#define METIS_VER_MAJOR 5
#define METIS_VER_MINOR 2
#define METIS_VER_SUBMINOR 1

/* The maximum length of the options[] array */
#define METIS_NOPTIONS 40


/*------------------------------------------------------------------------
* Function prototypes
*-------------------------------------------------------------------------*/

#ifdef _WINDLL
#define METIS_API(type) __declspec(dllexport) type __cdecl
#elif defined(__cdecl)
#define METIS_API(type) type __cdecl
#else
#define METIS_API(type) type
#endif


#ifdef __cplusplus
extern "C" {
#endif

METIS_API(int)
METIS_PartGraphRecursive(idx_t*  nvtxs,
                         idx_t*  ncon,
                         idx_t*  xadj,
                         idx_t*  adjncy,
                         idx_t*  vwgt,
                         idx_t*  vsize,
                         idx_t*  adjwgt,
                         idx_t*  nparts,
                         real_t* tpwgts,
                         real_t* ubvec,
                         idx_t*  options,
                         idx_t*  edgecut,
                         idx_t*  part);

METIS_API(int)
METIS_PartGraphKway(idx_t*  nvtxs,
                    idx_t*  ncon,
                    idx_t*  xadj,
                    idx_t*  adjncy,
                    idx_t*  vwgt,
                    idx_t*  vsize,
                    idx_t*  adjwgt,
                    idx_t*  nparts,
                    real_t* tpwgts,
                    real_t* ubvec,
                    idx_t*  options,
                    idx_t*  edgecut,
                    idx_t*  part);

METIS_API(int)
METIS_MeshToDual(idx_t*  ne,
                 idx_t*  nn,
                 idx_t*  eptr,
                 idx_t*  eind,
                 idx_t*  ncommon,
                 idx_t*  numflag,
                 idx_t** r_xadj,
                 idx_t** r_adjncy);

METIS_API(int)
METIS_MeshToNodal(idx_t* ne, idx_t* nn, idx_t* eptr, idx_t* eind, idx_t* numflag, idx_t** r_xadj, idx_t** r_adjncy);

METIS_API(int)
METIS_PartMeshNodal(idx_t*  ne,
                    idx_t*  nn,
                    idx_t*  eptr,
                    idx_t*  eind,
                    idx_t*  vwgt,
                    idx_t*  vsize,
                    idx_t*  nparts,
                    real_t* tpwgts,
                    idx_t*  options,
                    idx_t*  objval,
                    idx_t*  epart,
                    idx_t*  npart);

METIS_API(int)
METIS_PartMeshDual(idx_t*  ne,
                   idx_t*  nn,
                   idx_t*  eptr,
                   idx_t*  eind,
                   idx_t*  vwgt,
                   idx_t*  vsize,
                   idx_t*  ncommon,
                   idx_t*  nparts,
                   real_t* tpwgts,
                   idx_t*  options,
                   idx_t*  objval,
                   idx_t*  epart,
                   idx_t*  npart);

METIS_API(int)
METIS_NodeND(idx_t* nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* options, idx_t* perm, idx_t* iperm);

METIS_API(int) METIS_Free(void* ptr);

METIS_API(int) METIS_SetDefaultOptions(idx_t* options);


/* These functions are used by ParMETIS */

METIS_API(int)
METIS_NodeNDP(idx_t  nvtxs,
              idx_t* xadj,
              idx_t* adjncy,
              idx_t* vwgt,
              idx_t  npes,
              idx_t* options,
              idx_t* perm,
              idx_t* iperm,
              idx_t* sizes);

METIS_API(int)
METIS_ComputeVertexSeparator(idx_t* nvtxs,
                             idx_t* xadj,
                             idx_t* adjncy,
                             idx_t* vwgt,
                             idx_t* options,
                             idx_t* sepsize,
                             idx_t* part);

METIS_API(int)
METIS_NodeRefine(idx_t nvtxs, idx_t* xadj, idx_t* vwgt, idx_t* adjncy, idx_t* where, idx_t* hmarker, real_t ubfactor);


/* These functions are used by DGL */

METIS_API(int)
METIS_CacheFriendlyReordering(idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* part, idx_t* old2new);

#ifdef __cplusplus
}
#endif


/*------------------------------------------------------------------------
* Enum type definitions
*-------------------------------------------------------------------------*/
/*! Return codes */
typedef enum
{
    METIS_OK = 1,           /*!< Returned normally */
    METIS_ERROR_INPUT = -2, /*!< Returned due to erroneous inputs and/or options */
    METIS_ERROR_MEMORY = -3, /*!< Returned due to insufficient memory */
    METIS_ERROR        = -4  /*!< Some other errors */
} rstatus_et;


/*! Operation type codes */
typedef enum
{
    METIS_OP_PMETIS,
    METIS_OP_KMETIS,
    METIS_OP_OMETIS
} moptype_et;


/*! Options codes (i.e., options[]) */
typedef enum
{
    METIS_OPTION_PTYPE,
    METIS_OPTION_OBJTYPE,
    METIS_OPTION_CTYPE,
    METIS_OPTION_IPTYPE,
    METIS_OPTION_RTYPE,
    METIS_OPTION_DBGLVL,
    METIS_OPTION_NIPARTS,
    METIS_OPTION_NITER,
    METIS_OPTION_NCUTS,
    METIS_OPTION_SEED,
    METIS_OPTION_ONDISK,
    METIS_OPTION_MINCONN,
    METIS_OPTION_CONTIG,
    METIS_OPTION_COMPRESS,
    METIS_OPTION_CCORDER,
    METIS_OPTION_PFACTOR,
    METIS_OPTION_NSEPS,
    METIS_OPTION_UFACTOR,
    METIS_OPTION_NUMBERING,
    METIS_OPTION_DROPEDGES,
    METIS_OPTION_NO2HOP,
    METIS_OPTION_TWOHOP,
    METIS_OPTION_FAST,

    /* Used for command-line parameter purposes */
    METIS_OPTION_HELP,
    METIS_OPTION_TPWGTS,
    METIS_OPTION_NCOMMON,
    METIS_OPTION_NOOUTPUT,
    METIS_OPTION_BALANCE,
    METIS_OPTION_GTYPE,
    METIS_OPTION_UBVEC
} moptions_et;


/*! Partitioning Schemes */
typedef enum
{
    METIS_PTYPE_RB,
    METIS_PTYPE_KWAY
} mptype_et;

/*! Graph types for meshes */
typedef enum
{
    METIS_GTYPE_DUAL,
    METIS_GTYPE_NODAL
} mgtype_et;

/*! Coarsening Schemes */
typedef enum
{
    METIS_CTYPE_RM,
    METIS_CTYPE_SHEM
} mctype_et;

/*! Initial partitioning schemes */
typedef enum
{
    METIS_IPTYPE_GROW,
    METIS_IPTYPE_RANDOM,
    METIS_IPTYPE_EDGE,
    METIS_IPTYPE_NODE,
    METIS_IPTYPE_METISRB
} miptype_et;


/*! Refinement schemes */
typedef enum
{
    METIS_RTYPE_FM,
    METIS_RTYPE_GREEDY,
    METIS_RTYPE_SEP2SIDED,
    METIS_RTYPE_SEP1SIDED
} mrtype_et;


/*! Debug Levels */
typedef enum
{
    METIS_DBG_INFO     = 1,  /*!< Shows various diagnostic messages */
    METIS_DBG_TIME     = 2,  /*!< Perform timing analysis */
    METIS_DBG_COARSEN  = 4,  /*!< Show the coarsening progress */
    METIS_DBG_REFINE   = 8,  /*!< Show the refinement progress */
    METIS_DBG_IPART    = 16, /*!< Show info on initial partitioning */
    METIS_DBG_MOVEINFO = 32, /*!< Show info on vertex moves during refinement */
    METIS_DBG_SEPINFO = 64, /*!< Show info on vertex moves during sep refinement */
    METIS_DBG_CONNINFO = 128, /*!< Show info on minimization of subdomain connectivity */
    METIS_DBG_CONTIGINFO = 256, /*!< Show info on elimination of connected components */
    METIS_DBG_MEMORY = 2048 /*!< Show info related to wspace allocation */
} mdbglvl_et;


/* Types of objectives */
typedef enum
{
    METIS_OBJTYPE_CUT,
    METIS_OBJTYPE_VOL,
    METIS_OBJTYPE_NODE
} mobjtype_et;


/* Merged internal METIS headers */
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * rename.h
 *
 * This file contains header files
 *
 * Started 10/2/97
 * George
 *
 * $Id: rename.h 20398 2016-11-22 17:17:12Z karypis $
 *
 */


#ifndef _LIBMETIS_RENAME_H_
#define _LIBMETIS_RENAME_H_


/* balance.c */
#define Balance2Way libmetis__Balance2Way
#define Bnd2WayBalance libmetis__Bnd2WayBalance
#define General2WayBalance libmetis__General2WayBalance
#define McGeneral2WayBalance libmetis__McGeneral2WayBalance

/* bucketsort.c */
#define BucketSortKeysInc libmetis__BucketSortKeysInc

/* checkgraph.c */
#define CheckGraph libmetis__CheckGraph
#define CheckInputGraphWeights libmetis__CheckInputGraphWeights
#define FixGraph libmetis__FixGraph

/* coarsen.c */
#define CoarsenGraph libmetis__CoarsenGraph
#define Match_RM libmetis__Match_RM
#define Match_SHEM libmetis__Match_SHEM
#define Match_2Hop libmetis__Match_2Hop
#define Match_2HopAny libmetis__Match_2HopAny
#define Match_2HopAll libmetis__Match_2HopAll
#define Match_JC libmetis__Match_JC
#define PrintCGraphStats libmetis__PrintCGraphStats
#define CreateCoarseGraph libmetis__CreateCoarseGraph
#define SetupCoarseGraph libmetis__SetupCoarseGraph
#define ReAdjustMemory libmetis__ReAdjustMemory

/* compress.c */
#define CompressGraph libmetis__CompressGraph
#define PruneGraph libmetis__PruneGraph

/* contig.c */
#define FindPartitionInducedComponents libmetis__FindPartitionInducedComponents
#define IsConnected libmetis__IsConnected
#define IsConnectedSubdomain libmetis__IsConnectedSubdomain
#define FindSepInducedComponents libmetis__FindSepInducedComponents
#define EliminateComponents libmetis__EliminateComponents
#define MoveGroupContigForCut libmetis__MoveGroupContigForCut
#define MoveGroupContigForVol libmetis__MoveGroupContigForVol

/* debug.c */
#define ComputeCut libmetis__ComputeCut
#define ComputeVolume libmetis__ComputeVolume
#define ComputeMaxCut libmetis__ComputeMaxCut
#define CheckBnd libmetis__CheckBnd
#define CheckBnd2 libmetis__CheckBnd2
#define CheckNodeBnd libmetis__CheckNodeBnd
#define CheckRInfo libmetis__CheckRInfo
#define CheckNodePartitionParams libmetis__CheckNodePartitionParams
#define IsSeparable libmetis__IsSeparable
#define CheckKWayVolPartitionParams libmetis__CheckKWayVolPartitionParams

/* fm.c */
#define FM_2WayRefine libmetis__FM_2WayRefine
#define FM_2WayCutRefine libmetis__FM_2WayCutRefine
#define FM_Mc2WayCutRefine libmetis__FM_Mc2WayCutRefine
#define SelectQueue libmetis__SelectQueue
#define Print2WayRefineStats libmetis__Print2WayRefineStats

/* fortran.c */
#define Change2CNumbering libmetis__Change2CNumbering
#define Change2FNumbering libmetis__Change2FNumbering
#define Change2FNumbering2 libmetis__Change2FNumbering2
#define Change2FNumberingOrder libmetis__Change2FNumberingOrder
#define ChangeMesh2CNumbering libmetis__ChangeMesh2CNumbering
#define ChangeMesh2FNumbering libmetis__ChangeMesh2FNumbering
#define ChangeMesh2FNumbering2 libmetis__ChangeMesh2FNumbering2

/* graph.c */
#define SetupGraph libmetis__SetupGraph
#define SetupGraph_adjrsum libmetis__SetupGraph_adjrsum
#define SetupGraph_tvwgt libmetis__SetupGraph_tvwgt
#define SetupGraph_label libmetis__SetupGraph_label
#define SetupSplitGraph libmetis__SetupSplitGraph
#define CreateGraph libmetis__CreateGraph
#define InitGraph libmetis__InitGraph
#define FreeSData libmetis__FreeSData
#define FreeRData libmetis__FreeRData
#define FreeGraph libmetis__FreeGraph
#define graph_WriteToDisk libmetis__graph_WriteToDisk
#define graph_ReadFromDisk libmetis__graph_ReadFromDisk

/* initpart.c */
#define Init2WayPartition libmetis__Init2WayPartition
#define InitSeparator libmetis__InitSeparator
#define RandomBisection libmetis__RandomBisection
#define GrowBisection libmetis__GrowBisection
#define McRandomBisection libmetis__McRandomBisection
#define McGrowBisection libmetis__McGrowBisection
#define GrowBisectionNode libmetis__GrowBisectionNode

/* kmetis.c */
#define MlevelKWayPartitioning libmetis__MlevelKWayPartitioning
#define InitKWayPartitioning libmetis__InitKWayPartitioning

/* kwayfm.c */
#define Greedy_KWayOptimize libmetis__Greedy_KWayOptimize
#define Greedy_KWayCutOptimize libmetis__Greedy_KWayCutOptimize
#define Greedy_KWayVolOptimize libmetis__Greedy_KWayVolOptimize
#define Greedy_McKWayCutOptimize libmetis__Greedy_McKWayCutOptimize
#define Greedy_McKWayVolOptimize libmetis__Greedy_McKWayVolOptimize
#define IsArticulationNode libmetis__IsArticulationNode
#define KWayVolUpdate libmetis__KWayVolUpdate

/* kwayrefine.c */
#define RefineKWay libmetis__RefineKWay
#define AllocateKWayPartitionMemory libmetis__AllocateKWayPartitionMemory
#define ComputeKWayPartitionParams libmetis__ComputeKWayPartitionParams
#define ProjectKWayPartition libmetis__ProjectKWayPartition
#define ComputeKWayBoundary libmetis__ComputeKWayBoundary
#define ComputeKWayVolGains libmetis__ComputeKWayVolGains
#define IsBalanced libmetis__IsBalanced

/* mcutil */
#define rvecle libmetis__rvecle
#define rvecge libmetis__rvecge
#define rvecsumle libmetis__rvecsumle
#define rvecmaxdiff libmetis__rvecmaxdiff
#define ivecle libmetis__ivecle
#define ivecge libmetis__ivecge
#define ivecaxpylez libmetis__ivecaxpylez
#define ivecaxpygez libmetis__ivecaxpygez
#define BetterVBalance libmetis__BetterVBalance
#define BetterBalance2Way libmetis__BetterBalance2Way
#define BetterBalanceKWay libmetis__BetterBalanceKWay
#define ComputeLoadImbalance libmetis__ComputeLoadImbalance
#define ComputeLoadImbalanceDiff libmetis__ComputeLoadImbalanceDiff
#define ComputeLoadImbalanceDiffVec libmetis__ComputeLoadImbalanceDiffVec
#define ComputeLoadImbalanceVec libmetis__ComputeLoadImbalanceVec

/* mesh.c */
#define CreateGraphDual libmetis__CreateGraphDual
#define FindCommonElements libmetis__FindCommonElements
#define CreateGraphNodal libmetis__CreateGraphNodal
#define FindCommonNodes libmetis__FindCommonNodes
#define CreateMesh libmetis__CreateMesh
#define InitMesh libmetis__InitMesh
#define FreeMesh libmetis__FreeMesh

/* meshpart.c */
#define InduceRowPartFromColumnPart libmetis__InduceRowPartFromColumnPart

/* minconn.c */
#define ComputeSubDomainGraph libmetis__ComputeSubDomainGraph
#define UpdateEdgeSubDomainGraph libmetis__UpdateEdgeSubDomainGraph
#define PrintSubDomainGraph libmetis__PrintSubDomainGraph
#define EliminateSubDomainEdges libmetis__EliminateSubDomainEdges
#define MoveGroupMinConnForCut libmetis__MoveGroupMinConnForCut
#define MoveGroupMinConnForVol libmetis__MoveGroupMinConnForVol

/* mincover.c */
#define MinCover libmetis__MinCover
#define MinCover_Augment libmetis__MinCover_Augment
#define MinCover_Decompose libmetis__MinCover_Decompose
#define MinCover_ColDFS libmetis__MinCover_ColDFS
#define MinCover_RowDFS libmetis__MinCover_RowDFS

/* mmd.c */
#define genmmd libmetis__genmmd
#define mmdelm libmetis__mmdelm
#define mmdint libmetis__mmdint
#define mmdnum libmetis__mmdnum
#define mmdupd libmetis__mmdupd


/* ometis.c */
#define MlevelNestedDissection libmetis__MlevelNestedDissection
#define MlevelNestedDissectionCC libmetis__MlevelNestedDissectionCC
#define MlevelNodeBisectionMultiple libmetis__MlevelNodeBisectionMultiple
#define MlevelNodeBisectionL2 libmetis__MlevelNodeBisectionL2
#define MlevelNodeBisectionL1 libmetis__MlevelNodeBisectionL1
#define SplitGraphOrder libmetis__SplitGraphOrder
#define SplitGraphOrderCC libmetis__SplitGraphOrderCC
#define MMDOrder libmetis__MMDOrder

/* options.c */
#define SetupCtrl libmetis__SetupCtrl
#define SetupKWayBalMultipliers libmetis__SetupKWayBalMultipliers
#define Setup2WayBalMultipliers libmetis__Setup2WayBalMultipliers
#define PrintCtrl libmetis__PrintCtrl
#define FreeCtrl libmetis__FreeCtrl
#define CheckParams libmetis__CheckParams

/* parmetis.c */
#define MlevelNestedDissectionP libmetis__MlevelNestedDissectionP
#define FM_2WayNodeRefine1SidedP libmetis__FM_2WayNodeRefine1SidedP
#define FM_2WayNodeRefine2SidedP libmetis__FM_2WayNodeRefine2SidedP

/* pmetis.c */
#define MlevelRecursiveBisection libmetis__MlevelRecursiveBisection
#define MultilevelBisect libmetis__MultilevelBisect
#define SplitGraphPart libmetis__SplitGraphPart

/* refine.c */
#define Refine2Way libmetis__Refine2Way
#define Allocate2WayPartitionMemory libmetis__Allocate2WayPartitionMemory
#define Compute2WayPartitionParams libmetis__Compute2WayPartitionParams
#define Project2WayPartition libmetis__Project2WayPartition

/* separator.c */
#define ConstructSeparator libmetis__ConstructSeparator
#define ConstructMinCoverSeparator libmetis__ConstructMinCoverSeparator

/* sfm.c */
#define FM_2WayNodeRefine2Sided libmetis__FM_2WayNodeRefine2Sided
#define FM_2WayNodeRefine1Sided libmetis__FM_2WayNodeRefine1Sided
#define FM_2WayNodeBalance libmetis__FM_2WayNodeBalance

/* srefine.c */
#define Refine2WayNode libmetis__Refine2WayNode
#define Allocate2WayNodePartitionMemory                                        \
    libmetis__Allocate2WayNodePartitionMemory
#define Compute2WayNodePartitionParams libmetis__Compute2WayNodePartitionParams
#define Project2WayNodePartition libmetis__Project2WayNodePartition

/* stat.c */
#define ComputePartitionInfoBipartite libmetis__ComputePartitionInfoBipartite
#define ComputePartitionBalance libmetis__ComputePartitionBalance
#define ComputeElementBalance libmetis__ComputeElementBalance

/* timing.c */
#define InitTimers libmetis__InitTimers
#define PrintTimers libmetis__PrintTimers

/* util.c */
#define iargmax_strd libmetis__iargmax_strd
#define iargmax_nrm libmetis__iargmax_nrm
#define iargmax2_nrm libmetis__iargmax2_nrm
#define rargmax2 libmetis__rargmax2
#define InitRandom libmetis__InitRandom
#define metis_rcode libmetis__metis_rcode

/* wspace.c */
#define AllocateWorkSpace libmetis__AllocateWorkSpace
#define AllocateRefinementWorkSpace libmetis__AllocateRefinementWorkSpace
#define FreeWorkSpace libmetis__FreeWorkSpace
#define wspacemalloc libmetis__wspacemalloc
#define wspacepush libmetis__wspacepush
#define wspacepop libmetis__wspacepop
#define iwspacemalloc libmetis__iwspacemalloc
#define rwspacemalloc libmetis__rwspacemalloc
#define ikvwspacemalloc libmetis__ikvwspacemalloc
#define cnbrpoolReset libmetis__cnbrpoolReset
#define cnbrpoolGetNext libmetis__cnbrpoolGetNext
#define vnbrpoolReset libmetis__vnbrpoolReset
#define vnbrpoolGetNext libmetis__vnbrpoolGetNext

#endif


/*!
\file
\brief Data structures and prototypes for GKlib integration

\date  Started 12/23/2008
\author George
\version\verbatim $Id: gklib_defs.h 10395 2011-06-23 23:28:06Z karypis $ \endverbatim
*/

#ifndef _LIBMETIS_GKLIB_H_
#define _LIBMETIS_GKLIB_H_

/*!
\file

 * Copyright 1997, Regents of the University of Minnesota
 *
 * This file contains header files
 *
 * Started 10/2/97
 * George
 *
 * $Id: gklib_rename.h 10395 2011-06-23 23:28:06Z karypis $
 *
 */


#ifndef _LIBMETIS_GKLIB_RENAME_H_
#define _LIBMETIS_GKLIB_RENAME_H_

/* gklib.c - generated from the .o files using the ./utils/listundescapedsumbols.csh */
#define iAllocMatrix libmetis__iAllocMatrix
#define iFreeMatrix libmetis__iFreeMatrix
#define iSetMatrix libmetis__iSetMatrix
#define iargmax libmetis__iargmax
#define iargmax_n libmetis__iargmax_n
#define iargmin libmetis__iargmin
#define iarray2csr libmetis__iarray2csr
#define iaxpy libmetis__iaxpy
#define icopy libmetis__icopy
#define idot libmetis__idot
#define iincset libmetis__iincset
#define ikvAllocMatrix libmetis__ikvAllocMatrix
#define ikvFreeMatrix libmetis__ikvFreeMatrix
#define ikvSetMatrix libmetis__ikvSetMatrix
#define ikvcopy libmetis__ikvcopy
#define ikvmalloc libmetis__ikvmalloc
#define ikvrealloc libmetis__ikvrealloc
#define ikvset libmetis__ikvset
#define ikvsmalloc libmetis__ikvsmalloc
#define ikvsortd libmetis__ikvsortd
#define ikvsorti libmetis__ikvsorti
#define imalloc libmetis__imalloc
#define imax libmetis__imax
#define imin libmetis__imin
#define inorm2 libmetis__inorm2
#define ipqCheckHeap libmetis__ipqCheckHeap
#define ipqCreate libmetis__ipqCreate
#define ipqDelete libmetis__ipqDelete
#define ipqDestroy libmetis__ipqDestroy
#define ipqFree libmetis__ipqFree
#define ipqGetTop libmetis__ipqGetTop
#define ipqInit libmetis__ipqInit
#define ipqInsert libmetis__ipqInsert
#define ipqLength libmetis__ipqLength
#define ipqReset libmetis__ipqReset
#define ipqSeeKey libmetis__ipqSeeKey
#define ipqSeeTopKey libmetis__ipqSeeTopKey
#define ipqSeeTopVal libmetis__ipqSeeTopVal
#define ipqUpdate libmetis__ipqUpdate
#define isrand libmetis__isrand
#define irand libmetis__irand
#define irandArrayPermute libmetis__irandArrayPermute
#define irandArrayPermuteFine libmetis__irandArrayPermuteFine
#define irandInRange libmetis__irandInRange
#define irealloc libmetis__irealloc
#define iscale libmetis__iscale
#define iset libmetis__iset
#define ismalloc libmetis__ismalloc
#define isortd libmetis__isortd
#define isrand libmetis__isrand
#define isum libmetis__isum
#define rAllocMatrix libmetis__rAllocMatrix
#define rFreeMatrix libmetis__rFreeMatrix
#define rSetMatrix libmetis__rSetMatrix
#define rargmax libmetis__rargmax
#define rargmax_n libmetis__rargmax_n
#define rargmin libmetis__rargmin
#define raxpy libmetis__raxpy
#define rcopy libmetis__rcopy
#define rdot libmetis__rdot
#define rincset libmetis__rincset
#define rkvAllocMatrix libmetis__rkvAllocMatrix
#define rkvFreeMatrix libmetis__rkvFreeMatrix
#define rkvSetMatrix libmetis__rkvSetMatrix
#define rkvcopy libmetis__rkvcopy
#define rkvmalloc libmetis__rkvmalloc
#define rkvrealloc libmetis__rkvrealloc
#define rkvset libmetis__rkvset
#define rkvsmalloc libmetis__rkvsmalloc
#define rkvsortd libmetis__rkvsortd
#define rmalloc libmetis__rmalloc
#define rmax libmetis__rmax
#define rmin libmetis__rmin
#define rnorm2 libmetis__rnorm2
#define rpqCheckHeap libmetis__rpqCheckHeap
#define rpqCreate libmetis__rpqCreate
#define rpqDelete libmetis__rpqDelete
#define rpqDestroy libmetis__rpqDestroy
#define rpqFree libmetis__rpqFree
#define rpqGetTop libmetis__rpqGetTop
#define rpqInit libmetis__rpqInit
#define rpqInsert libmetis__rpqInsert
#define rpqLength libmetis__rpqLength
#define rpqReset libmetis__rpqReset
#define rpqSeeKey libmetis__rpqSeeKey
#define rpqSeeTopKey libmetis__rpqSeeTopKey
#define rpqSeeTopVal libmetis__rpqSeeTopVal
#define rpqUpdate libmetis__rpqUpdate
#define rrealloc libmetis__rrealloc
#define rscale libmetis__rscale
#define rset libmetis__rset
#define rsmalloc libmetis__rsmalloc
#define rsortd libmetis__rsortd
#define rsum libmetis__rsum
#define uvwsorti libmetis__uvwsorti

#endif


/*! Stores a weighted edge */
/*************************************************************************/
typedef struct
{
    idx_t u, v, w; /*!< Edge (u,v) with weight w */
} uvw_t;

/*************************************************************************
* Define various data structure using GKlib's templates.
**************************************************************************/
GK_MKKEYVALUE_T(ikv_t, idx_t, idx_t)
GK_MKKEYVALUE_T(rkv_t, real_t, idx_t)

#include "detail/pqueue.h"

using ipq_t = metis::detail::PQueue<idx_t, idx_t, ikv_t>;
using rpq_t = metis::detail::PQueue<real_t, idx_t, rkv_t>;


/* gklib.c */
GK_MKBLAS_PROTO(i, idx_t, idx_t)
GK_MKBLAS_PROTO(r, real_t, real_t)
GK_MKALLOC_PROTO(i, idx_t)
GK_MKALLOC_PROTO(r, real_t)
GK_MKALLOC_PROTO(ikv, ikv_t)
GK_MKALLOC_PROTO(rkv, rkv_t)

/* Priority queue API exposed as free functions that forward to the class
 * template in detail/pqueue.h.  These names are remapped to libmetis__ipq*
 * / libmetis__rpq* by the rename macros below. */
ipq_t* ipqCreate(size_t maxnodes);
void   ipqInit(ipq_t* queue, size_t maxnodes);
void   ipqReset(ipq_t* queue);
void   ipqFree(ipq_t* queue);
void   ipqDestroy(ipq_t* queue);
size_t ipqLength(ipq_t* queue);
int    ipqInsert(ipq_t* queue, idx_t node, idx_t key);
int    ipqDelete(ipq_t* queue, idx_t node);
void   ipqUpdate(ipq_t* queue, idx_t node, idx_t newkey);
idx_t  ipqGetTop(ipq_t* queue);
idx_t  ipqSeeTopVal(ipq_t* queue);
idx_t  ipqSeeTopKey(ipq_t* queue);
idx_t  ipqSeeKey(ipq_t* queue, idx_t node);
int    ipqCheckHeap(ipq_t* queue);

rpq_t* rpqCreate(size_t maxnodes);
void   rpqInit(rpq_t* queue, size_t maxnodes);
void   rpqReset(rpq_t* queue);
void   rpqFree(rpq_t* queue);
void   rpqDestroy(rpq_t* queue);
size_t rpqLength(rpq_t* queue);
int    rpqInsert(rpq_t* queue, idx_t node, real_t key);
int    rpqDelete(rpq_t* queue, idx_t node);
void   rpqUpdate(rpq_t* queue, idx_t node, real_t newkey);
idx_t  rpqGetTop(rpq_t* queue);
idx_t  rpqSeeTopVal(rpq_t* queue);
real_t rpqSeeTopKey(rpq_t* queue);
real_t rpqSeeKey(rpq_t* queue, idx_t node);
int    rpqCheckHeap(rpq_t* queue);

GK_MKRANDOM_PROTO(i, idx_t, idx_t)
GK_MKARRAY2CSR_PROTO(i, idx_t)
void isortd(size_t n, idx_t* base);
void ikvsorti(size_t n, ikv_t* base);
void ikvsortd(size_t n, ikv_t* base);
void rkvsortd(size_t n, rkv_t* base);
void uvwsorti(size_t n, uvw_t* base);

#endif
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * defs.h
 *
 * This file contains constant definitions
 *
 * Started 8/27/94
 * George
 *
 * $Id: defs.h 20398 2016-11-22 17:17:12Z karypis $
 *
 */

#ifndef _LIBMETIS_DEFS_H_
#define _LIBMETIS_DEFS_H_

#define METISTITLE                                                             \
    "METIS 5.2.1 Copyright 1998-22, Regents of the University of Minnesota\n"
#define MAXLINE 1280000

#define LTERM (void**)0 /* List terminator for gk_free() */

#define HTLENGTH ((1 << 13) - 1)

#define INIT_MAXNAD                                                            \
    200 /* Initial number of maximum number of
                                           adjacent domains. This number will be
                                           adjusted as required. */

/* Types of boundaries */
#define BNDTYPE_REFINE 1  /* Used for k-way refinement-purposes */
#define BNDTYPE_BALANCE 2 /* Used for k-way balancing purposes */

/* Mode of optimization */
#define OMODE_REFINE 1  /* Optimize the objective function */
#define OMODE_BALANCE 2 /* Balance the subdomains */

/* Types of vertex statues in the priority queue */
#define VPQSTATUS_PRESENT 1   /* The vertex is in the queue */
#define VPQSTATUS_EXTRACTED 2 /* The vertex has been extracted from the queue */
#define VPQSTATUS_NOTPRESENT                                                   \
    3 /* The vertex is not present in the queue and
                                          has not been extracted before */

#define UNMATCHED -1

#define LARGENIPARTS 7 /* Number of random initial partitions */
#define SMALLNIPARTS 5 /* Number of random initial partitions */

#define COARSEN_FRACTION                                                       \
    0.85 /* Node reduction between successive coarsening levels */

#define COMPRESSION_FRACTION 0.85

#define MMDSWITCH 120

/* Default ufactors for the various operational modes */
#define PMETIS_DEFAULT_UFACTOR 1
#define MCPMETIS_DEFAULT_UFACTOR 10
#define KMETIS_DEFAULT_UFACTOR 30
#define OMETIS_DEFAULT_UFACTOR 200

#endif
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * struct.h
 *
 * This file contains data structures for ILU routines.
 *
 * Started 9/26/95
 * George
 *
 * $Id: struct.h 14362 2013-05-21 21:35:23Z karypis $
 */

#ifndef _LIBMETIS_STRUCT_H_
#define _LIBMETIS_STRUCT_H_


/*************************************************************************/
/*! This data structure stores cut-based k-way refinement info about an
    adjacent subdomain for a given vertex. */
/*************************************************************************/
typedef struct cnbr_t
{
    idx_t pid; /*!< The partition ID */
    idx_t ed;  /*!< The sum of the weights of the adjacent edges
                             that are incident on pid */
} cnbr_t;


/*************************************************************************/
/*! The following data structure stores holds information on degrees for k-way
    partition */
/*************************************************************************/
typedef struct ckrinfo_t
{
    idx_t id;    /*!< The internal degree of a vertex (sum of weights) */
    idx_t ed;    /*!< The total external degree of a vertex */
    idx_t nnbrs; /*!< The number of neighboring subdomains */
    idx_t inbr;  /*!< The index in the cnbr_t array where the nnbrs list
                             of neighbors is stored */
} ckrinfo_t;


/*************************************************************************/
/*! This data structure stores volume-based k-way refinement info about an
    adjacent subdomain for a given vertex. */
/*************************************************************************/
typedef struct vnbr_t
{
    idx_t pid; /*!< The partition ID */
    idx_t ned; /*!< The number of the adjacent edges
                             that are incident on pid */
    idx_t gv;  /*!< The gain in volume achieved by moving the
                             vertex to pid */
} vnbr_t;


/*************************************************************************/
/*! The following data structure holds information on degrees for k-way
    vol-based partition */
/*************************************************************************/
typedef struct vkrinfo_t
{
    idx_t nid;   /*!< The internal degree of a vertex (count of edges) */
    idx_t ned;   /*!< The total external degree of a vertex (count of edges) */
    idx_t gv;    /*!< The volume gain of moving that vertex */
    idx_t nnbrs; /*!< The number of neighboring subdomains */
    idx_t inbr;  /*!< The index in the vnbr_t array where the nnbrs list
                             of neighbors is stored */
} vkrinfo_t;


/*************************************************************************/
/*! The following data structure holds information on degrees for k-way
    partition */
/*************************************************************************/
typedef struct nrinfo_t
{
    idx_t edegrees[2];
} nrinfo_t;


/*************************************************************************/
/*! This data structure holds a graph */
/*************************************************************************/
typedef struct graph_t
{
    idx_t  nvtxs, nedges; /* The # of vertices and edges in the graph */
    idx_t  ncon;          /* The # of constrains */
    idx_t* xadj;          /* Pointers to the locally stored vertices */
    idx_t* vwgt;          /* Vertex weights */
    idx_t* vsize;         /* Vertex sizes for min-volume formulation */
    idx_t* adjncy;        /* Array that stores the adjacency lists of nvtxs */
    idx_t* adjwgt; /* Array that stores the weights of the adjacency lists */

    idx_t* tvwgt; /* The sum of the vertex weights in the graph */
    real_t* invtvwgt; /* The inverse of the sum of the vertex weights in the graph */


    /* These are to keep track control if the corresponding fields correspond to
     application or library memory */
    int free_xadj, free_vwgt, free_vsize, free_adjncy, free_adjwgt;

    idx_t* cmap; /* The contraction/coarsening map */

    idx_t* label; /* The labels of the vertices for recursive bisection (pmetis/ometis) */

    /* Partition parameters */
    idx_t  mincut, minvol;
    idx_t *where, *pwgts;
    idx_t  nbnd;
    idx_t *bndptr, *bndind;

    /* Bisection refinement parameters */
    idx_t *id, *ed;

    /* K-way refinement parameters */
    ckrinfo_t* ckrinfo; /*!< The per-vertex cut-based refinement info */
    vkrinfo_t* vkrinfo; /*!< The per-vertex volume-based refinement info */

    /* Node refinement information */
    nrinfo_t* nrinfo;

    /* various fields for out-of-core processing */
    int gID;
    int ondisk;

    /* keep track of the dropped edgewgt */
    idx_t droppedewgt;

    /* the linked-list structure of the sequence of graphs */
    struct graph_t *coarser, *finer;

} graph_t;


/*************************************************************************/
/*! This data structure holds a mesh */
/*************************************************************************/
typedef struct mesh_t
{
    idx_t ne, nn; /*!< The # of elements and nodes in the mesh */
    idx_t ncon; /*!< The number of element balancing constraints (element weights) */

    idx_t *eptr, *eind; /*!< The CSR-structure storing the nodes in the elements */
    idx_t* ewgt;        /*!< The weights of the elements */
} mesh_t;


/*************************************************************************/
/*! The following structure stores information used by Metis */
/*************************************************************************/
typedef struct ctrl_t
{
    idx_t optype;  /* Type of operation (moptype_et) */
    idx_t objtype; /* Type of refinement objective (mobjtype_et) */
    idx_t dbglvl; /* Controls the debugging output of the program (mdbglvl_et) */
    idx_t ctype;  /* The type of coarsening (mctype_et) */
    idx_t iptype; /* The type of initial partitioning (miptype_et) */
    idx_t rtype;  /* The type of refinement (mrtype_et) */

    idx_t CoarsenTo; /* The # of vertices in the coarsest graph */
    idx_t nIparts;   /* The number of initial partitions to compute */
    idx_t no2hop;    /* Indicates if 2-hop matching will be used */
    idx_t ondisk;    /* Indicates out-of-core execution */
    idx_t minconn; /* Indicates if the subdomain connectivity will be minimized */
    idx_t contig; /* Indicates if contiguous partitions are required */
    idx_t nseps; /* The number of separators to be found during multiple bisections */
    idx_t ufactor;  /* The user-supplied load imbalance factor */
    idx_t compress; /* If the graph will be compressed prior to ordering */
    idx_t ccorder;  /* If connected components will be ordered separately */
    idx_t seed;     /* The seed for the random number generator */
    idx_t ncuts;    /* The number of different partitionings to compute */
    idx_t niter;    /* The number of iterations during each refinement */
    idx_t numflag;  /* The user-supplied numflag for the graph */
    idx_t dropedges; /* Indicates if edges will be randomly dropped during coarsening */
    idx_t* maxvwgt; /* The maximum allowed weight for a vertex */

    idx_t ncon;   /*!< The number of balancing constraints */
    idx_t nparts; /*!< The number of partitions */

    real_t pfactor; /* .1*(user-supplied prunning factor) */

    real_t* ubfactors; /*!< The per-constraint ubfactors */

    real_t* tpwgts; /*!< The target partition weights */
    real_t* pijbm;  /*!< The nparts*ncon multiplies for the ith partition
                                     and jth constraint for obtaining the balance */

    real_t cfactor; /*!< The achieved compression factor */

    /* Various Timers */
    double TotalTmr, InitPartTmr, MatchTmr, ContractTmr, CoarsenTmr,
        UncoarsenTmr, RefTmr, ProjectTmr, SplitTmr, Aux1Tmr, Aux2Tmr, Aux3Tmr;

    /* Workspace information */
    gk_mcore_t* mcore; /*!< The persistent memory core for within function
                             mallocs/frees */

    /* These are for use by the k-way refinement routines */
    size_t nbrpoolsize_max; /*!< The maximum number of {c,v}nbr_t entries that will ever be allocated */
    size_t nbrpoolsize; /*!< The number of {c,v}nbr_t entries that have been allocated */
    size_t nbrpoolcpos; /*!< The position of the first free entry in the array */
    size_t nbrpoolreallocs; /*!< The number of times the pool was resized */

    cnbr_t* cnbrpool; /*!< The pool of cnbr_t entries to be used during refinement.
                             The size and current position of the pool is controlled
                             by nnbrs & cnbrs */
    vnbr_t* vnbrpool; /*!< The pool of vnbr_t entries to be used during refinement.
                             The size and current position of the pool is controlled
                             by nnbrs & cnbrs */

    /* The subdomain graph, in sparse format  */
    idx_t*  maxnads; /* The maximum allocated number of adjacent domains */
    idx_t*  nads;    /* The number of adjacent domains */
    idx_t** adids;   /* The IDs of the adjacent domains */
    idx_t** adwgts;  /* The edge-weight to the adjacent domains */
    idx_t * pvec1, *pvec2; /* Auxiliary nparts-size vectors for efficiency */

    /* ondisk related info */
    pid_t pid; /*!< The pid of the running process */
} ctrl_t;


#endif
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * macros.h
 *
 * This file contains macros used in multilevel
 *
 * Started 9/25/94
 * George
 *
 * $Id: macros.h 10060 2011-06-02 18:56:30Z karypis $
 *
 */

#ifndef _LIBMETIS_MACROS_H_
#define _LIBMETIS_MACROS_H_

/*************************************************************************
* The following macro returns a random number in the specified range
**************************************************************************/
#define AND(a, b) ((a) < 0 ? ((-(a)) & (b)) : ((a) & (b)))
#define OR(a, b) ((a) < 0 ? -((-(a)) | (b)) : ((a) | (b)))
#define XOR(a, b) ((a) < 0 ? -((-(a)) ^ (b)) : ((a) ^ (b)))

//#define icopy(n, a, b) (idx_t *)memcpy((void *)(b), (void *)(a), sizeof(idx_t)*(n))

#define HASHFCT(key, size) ((key) % (size))
#define SWAP gk_SWAP

/* gets the appropriate option value */
#define GETOPTION(options, idx, defval)                                        \
    ((options) == NULL || (options)[idx] == -1 ? defval : (options)[idx])

/* converts a user provided ufactor into a real ubfactor */
#define I2RUBFACTOR(ufactor) (1.0 + 0.001 * (ufactor))

/* set/reset the current workspace core */
#define WCOREPUSH wspacepush(ctrl)
#define WCOREPOP wspacepop(ctrl)


/*************************************************************************
* These macros insert and remove nodes from a Direct Access list
**************************************************************************/
#define ListInsert(n, lind, lptr, i)                                           \
    do                                                                         \
    {                                                                          \
        ASSERT(lptr[i] == -1);                                                 \
        lind[n] = i;                                                           \
        lptr[i] = (n)++;                                                       \
    } while(0)

#define ListDelete(n, lind, lptr, i)                                           \
    do                                                                         \
    {                                                                          \
        ASSERT(lptr[i] != -1);                                                 \
        lind[lptr[i]] = lind[--(n)];                                           \
        lptr[lind[n]] = lptr[i];                                               \
        lptr[i]       = -1;                                                    \
    } while(0)


/*************************************************************************
* These macros insert and remove nodes from the boundary list
**************************************************************************/
#define BNDInsert(nbnd, bndind, bndptr, vtx)                                   \
    ListInsert(nbnd, bndind, bndptr, vtx)

#define BNDDelete(nbnd, bndind, bndptr, vtx)                                   \
    ListDelete(nbnd, bndind, bndptr, vtx)


/*************************************************************************
* These macros deal with id/ed updating during k-way refinement
**************************************************************************/
#define UpdateMovedVertexInfoAndBND(                                           \
    i, from, k, to, myrinfo, mynbrs, where, nbnd, bndptr, bndind, bndtype)     \
    do                                                                         \
    {                                                                          \
        where[i] = to;                                                         \
        myrinfo->ed += myrinfo->id - mynbrs[k].ed;                             \
        SWAP(myrinfo->id, mynbrs[k].ed, j);                                    \
        if(mynbrs[k].ed == 0)                                                  \
            mynbrs[k] = mynbrs[--myrinfo->nnbrs];                              \
        else                                                                   \
            mynbrs[k].pid = from;                                              \
                                                                               \
        /* Update the boundary information. Both deletion and addition is \
        allowed as this routine can be used for moving arbitrary nodes. */    \
        if(bndtype == BNDTYPE_REFINE)                                          \
        {                                                                      \
            if(bndptr[i] != -1 && myrinfo->ed - myrinfo->id < 0)               \
                BNDDelete(nbnd, bndind, bndptr, i);                            \
            if(bndptr[i] == -1 && myrinfo->ed - myrinfo->id >= 0)              \
                BNDInsert(nbnd, bndind, bndptr, i);                            \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            if(bndptr[i] != -1 && myrinfo->ed <= 0)                            \
                BNDDelete(nbnd, bndind, bndptr, i);                            \
            if(bndptr[i] == -1 && myrinfo->ed > 0)                             \
                BNDInsert(nbnd, bndind, bndptr, i);                            \
        }                                                                      \
    } while(0)


#define UpdateAdjacentVertexInfoAndBND(                                            \
    ctrl, vid, adjlen, me, from, to, myrinfo, ewgt, nbnd, bndptr, bndind, bndtype) \
    do                                                                             \
    {                                                                              \
        idx_t   k;                                                                 \
        cnbr_t* mynbrs;                                                            \
                                                                                   \
        if(myrinfo->inbr == -1)                                                    \
        {                                                                          \
            myrinfo->inbr  = cnbrpoolGetNext(ctrl, adjlen);                        \
            myrinfo->nnbrs = 0;                                                    \
        }                                                                          \
        ASSERT(CheckRInfo(ctrl, myrinfo));                                         \
                                                                                   \
        mynbrs = ctrl->cnbrpool + myrinfo->inbr;                                   \
                                                                                   \
        /* Update global ID/ED and boundary */                                     \
        if(me == from)                                                             \
        {                                                                          \
            INC_DEC(myrinfo->ed, myrinfo->id, (ewgt));                             \
            if(bndtype == BNDTYPE_REFINE)                                          \
            {                                                                      \
                if(myrinfo->ed - myrinfo->id >= 0 && bndptr[(vid)] == -1)          \
                    BNDInsert(nbnd, bndind, bndptr, (vid));                        \
            }                                                                      \
            else                                                                   \
            {                                                                      \
                if(myrinfo->ed > 0 && bndptr[(vid)] == -1)                         \
                    BNDInsert(nbnd, bndind, bndptr, (vid));                        \
            }                                                                      \
        }                                                                          \
        else if(me == to)                                                          \
        {                                                                          \
            INC_DEC(myrinfo->id, myrinfo->ed, (ewgt));                             \
            if(bndtype == BNDTYPE_REFINE)                                          \
            {                                                                      \
                if(myrinfo->ed - myrinfo->id < 0 && bndptr[(vid)] != -1)           \
                    BNDDelete(nbnd, bndind, bndptr, (vid));                        \
            }                                                                      \
            else                                                                   \
            {                                                                      \
                if(myrinfo->ed <= 0 && bndptr[(vid)] != -1)                        \
                    BNDDelete(nbnd, bndind, bndptr, (vid));                        \
            }                                                                      \
        }                                                                          \
                                                                                   \
        /* Remove contribution from the .ed of 'from' */                           \
        if(me != from)                                                             \
        {                                                                          \
            for(k = 0; k < myrinfo->nnbrs; k++)                                    \
            {                                                                      \
                if(mynbrs[k].pid == from)                                          \
                {                                                                  \
                    if(mynbrs[k].ed == (ewgt))                                     \
                        mynbrs[k] = mynbrs[--myrinfo->nnbrs];                      \
                    else                                                           \
                        mynbrs[k].ed -= (ewgt);                                    \
                    break;                                                         \
                }                                                                  \
            }                                                                      \
        }                                                                          \
                                                                                   \
        /* Add contribution to the .ed of 'to' */                                  \
        if(me != to)                                                               \
        {                                                                          \
            for(k = 0; k < myrinfo->nnbrs; k++)                                    \
            {                                                                      \
                if(mynbrs[k].pid == to)                                            \
                {                                                                  \
                    mynbrs[k].ed += (ewgt);                                        \
                    break;                                                         \
                }                                                                  \
            }                                                                      \
            if(k == myrinfo->nnbrs)                                                \
            {                                                                      \
                mynbrs[k].pid = to;                                                \
                mynbrs[k].ed  = (ewgt);                                            \
                myrinfo->nnbrs++;                                                  \
            }                                                                      \
        }                                                                          \
                                                                                   \
        ASSERT(CheckRInfo(ctrl, myrinfo));                                         \
    } while(0)


#define UpdateQueueInfo(                                                                  \
    queue, vstatus, vid, me, from, to, myrinfo, oldnnbrs, nupd, updptr, updind, bndtype)  \
    do                                                                                    \
    {                                                                                     \
        real_t rgain;                                                                     \
                                                                                          \
        if(me == to || me == from || oldnnbrs != myrinfo->nnbrs)                          \
        {                                                                                 \
            rgain = (myrinfo->nnbrs > 0 ? 1.0 * myrinfo->ed / sqrt(myrinfo->nnbrs) : 0.0) \
                    - myrinfo->id;                                                        \
                                                                                          \
            if(bndtype == BNDTYPE_REFINE)                                                 \
            {                                                                             \
                if(vstatus[(vid)] == VPQSTATUS_PRESENT)                                   \
                {                                                                         \
                    if(myrinfo->ed - myrinfo->id >= 0)                                    \
                        rpqUpdate(queue, (vid), rgain);                                   \
                    else                                                                  \
                    {                                                                     \
                        rpqDelete(queue, (vid));                                          \
                        vstatus[(vid)] = VPQSTATUS_NOTPRESENT;                            \
                        ListDelete(nupd, updind, updptr, (vid));                          \
                    }                                                                     \
                }                                                                         \
                else if(vstatus[(vid)] == VPQSTATUS_NOTPRESENT                            \
                        && myrinfo->ed - myrinfo->id >= 0)                                \
                {                                                                         \
                    rpqInsert(queue, (vid), rgain);                                       \
                    vstatus[(vid)] = VPQSTATUS_PRESENT;                                   \
                    ListInsert(nupd, updind, updptr, (vid));                              \
                }                                                                         \
            }                                                                             \
            else                                                                          \
            {                                                                             \
                if(vstatus[(vid)] == VPQSTATUS_PRESENT)                                   \
                {                                                                         \
                    if(myrinfo->ed > 0)                                                   \
                        rpqUpdate(queue, (vid), rgain);                                   \
                    else                                                                  \
                    {                                                                     \
                        rpqDelete(queue, (vid));                                          \
                        vstatus[(vid)] = VPQSTATUS_NOTPRESENT;                            \
                        ListDelete(nupd, updind, updptr, (vid));                          \
                    }                                                                     \
                }                                                                         \
                else if(vstatus[(vid)] == VPQSTATUS_NOTPRESENT && myrinfo->ed > 0)        \
                {                                                                         \
                    rpqInsert(queue, (vid), rgain);                                       \
                    vstatus[(vid)] = VPQSTATUS_PRESENT;                                   \
                    ListInsert(nupd, updind, updptr, (vid));                              \
                }                                                                         \
            }                                                                             \
        }                                                                                 \
    } while(0)


/*************************************************************************/
/*! This macro determines the set of subdomains that a vertex can move to
    without increasins the maxndoms. */
/*************************************************************************/
#define SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, vtmp) \
    do                                                                                    \
    {                                                                                     \
        idx_t j, k, l, nadd, to;                                                          \
        for(j = 0; j < myrinfo->nnbrs; j++)                                               \
        {                                                                                 \
            safetos[to = mynbrs[j].pid] = 0;                                              \
                                                                                          \
            /* uncompress the connectivity info for the 'to' subdomain */                 \
            for(k = 0; k < nads[to]; k++)                                                 \
                vtmp[adids[to][k]] = 1;                                                   \
                                                                                          \
            for(nadd = 0, k = 0; k < myrinfo->nnbrs; k++)                                 \
            {                                                                             \
                if(k == j)                                                                \
                    continue;                                                             \
                                                                                          \
                l = mynbrs[k].pid;                                                        \
                if(vtmp[l] == 0)                                                          \
                {                                                                         \
                    if(nads[l] > maxndoms - 1)                                            \
                    {                                                                     \
                        nadd = maxndoms;                                                  \
                        break;                                                            \
                    }                                                                     \
                    nadd++;                                                               \
                }                                                                         \
            }                                                                             \
            if(nads[to] + nadd <= maxndoms)                                               \
                safetos[to] = 1;                                                          \
            if(nadd == 0)                                                                 \
                safetos[to] = 2;                                                          \
                                                                                          \
            /* cleanup the connectivity info due to the 'to' subdomain */                 \
            for(k = 0; k < nads[to]; k++)                                                 \
                vtmp[adids[to][k]] = 0;                                                   \
        }                                                                                 \
    } while(0)


#endif
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * proto.h
 *
 * This file contains header files
 *
 * Started 10/19/95
 * George
 *
 * $Id: proto.h 20398 2016-11-22 17:17:12Z karypis $
 *
 */

#ifndef _LIBMETIS_PROTO_H_
#define _LIBMETIS_PROTO_H_

/* auxapi.c */

/* balance.c */
void Balance2Way(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts);
void Bnd2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts);
void General2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts);
void McGeneral2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts);


/* bucketsort.c */
void BucketSortKeysInc(ctrl_t* ctrl, idx_t n, idx_t max, idx_t* keys, idx_t* tperm, idx_t* perm);


/* checkgraph.c */
int CheckGraph(graph_t* graph, int numflag, int verbose);
int CheckInputGraphWeights(
    idx_t nvtxs, idx_t ncon, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt);
graph_t* FixGraph(graph_t* graph);


/* coarsen.c */
graph_t* CoarsenGraph(ctrl_t* ctrl, graph_t* graph);
graph_t* CoarsenGraphNlevels(ctrl_t* ctrl, graph_t* graph, idx_t nlevels);
idx_t    Match_RM(ctrl_t* ctrl, graph_t* graph);
idx_t    Match_SHEM(ctrl_t* ctrl, graph_t* graph);
idx_t Match_2Hop(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match, idx_t cnvtxs, size_t nunmatched);
idx_t Match_2HopAny(ctrl_t*  ctrl,
                    graph_t* graph,
                    idx_t*   perm,
                    idx_t*   match,
                    idx_t    cnvtxs,
                    size_t*  r_nunmatched,
                    size_t   maxdegree);
idx_t Match_2HopAll(ctrl_t*  ctrl,
                    graph_t* graph,
                    idx_t*   perm,
                    idx_t*   match,
                    idx_t    cnvtxs,
                    size_t*  r_nunmatched,
                    size_t   maxdegree);
idx_t Match_JC(ctrl_t* ctrl, graph_t* graph);
void  PrintCGraphStats(ctrl_t* ctrl, graph_t* graph);
void CreateCoarseGraph(ctrl_t* ctrl, graph_t* graph, idx_t cnvtxs, idx_t* match);
graph_t* SetupCoarseGraph(graph_t* graph, idx_t cnvtxs, int dovsize);
void     ReAdjustMemory(ctrl_t* ctrl, graph_t* graph, graph_t* cgraph);


/* compress.c */
graph_t* CompressGraph(
    ctrl_t* ctrl, idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* cptr, idx_t* cind);
graph_t* PruneGraph(ctrl_t* ctrl, idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* iperm, real_t factor);


/* contig.c */
idx_t FindPartitionInducedComponents(graph_t* graph, idx_t* where, idx_t* cptr, idx_t* cind);
void  ComputeBFSOrdering(ctrl_t* ctrl, graph_t* graph, idx_t* bfsperm);
idx_t IsConnected(graph_t* graph, idx_t report);
idx_t IsConnectedSubdomain(ctrl_t*, graph_t*, idx_t, idx_t);
idx_t FindSepInducedComponents(ctrl_t*, graph_t*, idx_t*, idx_t*);
void  EliminateComponents(ctrl_t* ctrl, graph_t* graph);
void MoveGroupContigForCut(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t gid, idx_t* ptr, idx_t* ind);
void MoveGroupContigForVol(ctrl_t*  ctrl,
                           graph_t* graph,
                           idx_t    to,
                           idx_t    gid,
                           idx_t*   ptr,
                           idx_t*   ind,
                           idx_t*   vmarker,
                           idx_t*   pmarker,
                           idx_t*   modind);


/* debug.c */
idx_t ComputeCut(graph_t* graph, idx_t* where);
idx_t ComputeVolume(graph_t*, idx_t*);
idx_t ComputeMaxCut(graph_t* graph, idx_t nparts, idx_t* where);
idx_t CheckBnd(graph_t*);
idx_t CheckBnd2(graph_t*);
idx_t CheckNodeBnd(graph_t*, idx_t);
idx_t CheckRInfo(ctrl_t* ctrl, ckrinfo_t* rinfo);
idx_t CheckNodePartitionParams(graph_t*);
idx_t IsSeparable(graph_t*);
void  CheckKWayVolPartitionParams(ctrl_t* ctrl, graph_t* graph);


/* fm.c */
void FM_2WayRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter);
void FM_2WayCutRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter);
void FM_Mc2WayCutRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter);
void SelectQueue(graph_t* graph, real_t* pijbm, real_t* ubfactors, rpq_t** queues, idx_t* from, idx_t* cnum);
void Print2WayRefineStats(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, real_t deltabal, idx_t mincutorder);


/* fortran.c */
void Change2CNumbering(idx_t, idx_t*, idx_t*);
void Change2FNumbering(idx_t, idx_t*, idx_t*, idx_t*);
void Change2FNumbering2(idx_t, idx_t*, idx_t*);
void Change2FNumberingOrder(idx_t, idx_t*, idx_t*, idx_t*, idx_t*);
void ChangeMesh2CNumbering(idx_t n, idx_t* ptr, idx_t* ind);
void ChangeMesh2FNumbering(idx_t n, idx_t* ptr, idx_t* ind, idx_t nvtxs, idx_t* xadj, idx_t* adjncy);
void ChangeMesh2FNumbering2(idx_t ne, idx_t nn, idx_t* ptr, idx_t* ind, idx_t* epart, idx_t* npart);


/* graph.c */
graph_t* SetupGraph(ctrl_t* ctrl,
                    idx_t   nvtxs,
                    idx_t   ncon,
                    idx_t*  xadj,
                    idx_t*  adjncy,
                    idx_t*  vwgt,
                    idx_t*  vsize,
                    idx_t*  adjwgt);
void     SetupGraph_tvwgt(graph_t* graph);
void     SetupGraph_label(graph_t* graph);
graph_t* SetupSplitGraph(graph_t* graph, idx_t snvtxs, idx_t snedges);
graph_t* CreateGraph(void);
void     InitGraph(graph_t* graph);
void     FreeSData(graph_t* graph);
void     FreeRData(graph_t* graph);
void     FreeGraph(graph_t** graph);
void     graph_WriteToDisk(ctrl_t* ctrl, graph_t* graph);
void     graph_ReadFromDisk(ctrl_t* ctrl, graph_t* graph);


/* initpart.c */
void Init2WayPartition(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);
void InitSeparator(ctrl_t* ctrl, graph_t* graph, idx_t niparts);
void RandomBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);
void GrowBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);
void McRandomBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);
void McGrowBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);
void GrowBisectionNode(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);
void GrowBisectionNode2(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts);


/* kmetis.c */
idx_t MlevelKWayPartitioning(ctrl_t* ctrl, graph_t* graph, idx_t* part);
void  InitKWayPartitioning(ctrl_t* ctrl, graph_t* graph);
idx_t BlockKWayPartitioning(ctrl_t* ctrl, graph_t* graph, idx_t* part);
idx_t GrowMultisection(ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* where);
void BalanceAndRefineLP(ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* where);


/* kwayfm.c */
void Greedy_KWayOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode);
void Greedy_KWayCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode);
void Greedy_KWayVolOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode);
void Greedy_McKWayCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode);
void Greedy_McKWayVolOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode);
idx_t IsArticulationNode(
    idx_t i, idx_t* xadj, idx_t* adjncy, idx_t* where, idx_t* bfslvl, idx_t* bfsind, idx_t* bfsmrk);
void KWayVolUpdate(ctrl_t*  ctrl,
                   graph_t* graph,
                   idx_t    v,
                   idx_t    from,
                   idx_t    to,
                   ipq_t*   queue,
                   idx_t*   vstatus,
                   idx_t*   r_nupd,
                   idx_t*   updptr,
                   idx_t*   updind,
                   idx_t    bndtype,
                   idx_t*   vmarker,
                   idx_t*   pmarker,
                   idx_t*   modind);
void Greedy_KWayEdgeStats(ctrl_t* ctrl, graph_t* graph);
void Greedy_KWayEdgeCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter);


/* kwayrefine.c */
void RefineKWay(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph);
void AllocateKWayPartitionMemory(ctrl_t* ctrl, graph_t* graph);
void ComputeKWayPartitionParams(ctrl_t* ctrl, graph_t* graph);
void ProjectKWayPartition(ctrl_t* ctrl, graph_t* graph);
void ComputeKWayBoundary(ctrl_t* ctrl, graph_t* graph, idx_t bndtype);
void ComputeKWayVolGains(ctrl_t* ctrl, graph_t* graph);
int  IsBalanced(ctrl_t* ctrl, graph_t* graph, real_t ffactor);


/* mcutil.c */
int    rvecle(idx_t n, real_t* x, real_t* y);
int    rvecge(idx_t n, real_t* x, real_t* y);
int    rvecsumle(idx_t n, real_t* x1, real_t* x2, real_t* y);
real_t rvecmaxdiff(idx_t n, real_t* x, real_t* y);
int    ivecle(idx_t n, idx_t* x, idx_t* z);
int    ivecge(idx_t n, idx_t* x, idx_t* z);
int    ivecaxpylez(idx_t n, idx_t a, idx_t* x, idx_t* y, idx_t* z);
int    ivecaxpygez(idx_t n, idx_t a, idx_t* x, idx_t* y, idx_t* z);
int BetterVBalance(idx_t ncon, real_t* itvwgt, idx_t* v_vwgt, idx_t* u1_vwgt, idx_t* u2_vwgt);
int    BetterBalance2Way(idx_t n, real_t* x, real_t* y);
int    BetterBalanceKWay(idx_t   ncon,
                         idx_t*  vwgt,
                         real_t* itvwgt,
                         idx_t   a1,
                         idx_t*  pt1,
                         real_t* bm1,
                         idx_t   a2,
                         idx_t*  pt2,
                         real_t* bm2);
real_t ComputeLoadImbalance(graph_t* graph, idx_t nparts, real_t* pijbm);
real_t ComputeLoadImbalanceDiff(graph_t* graph, idx_t nparts, real_t* pijbm, real_t* ubvec);
real_t ComputeLoadImbalanceDiffVec(
    graph_t* graph, idx_t nparts, real_t* pijbm, real_t* ubfactors, real_t* diffvec);
void ComputeLoadImbalanceVec(graph_t* graph, idx_t nparts, real_t* pijbm, real_t* lbvec);


/* mesh.c */
void CreateGraphDual(
    idx_t ne, idx_t nn, idx_t* eptr, idx_t* eind, idx_t ncommon, idx_t** r_xadj, idx_t** r_adjncy);
idx_t FindCommonElements(idx_t  qid,
                         idx_t  elen,
                         idx_t* eind,
                         idx_t* nptr,
                         idx_t* nind,
                         idx_t* eptr,
                         idx_t  ncommon,
                         idx_t* marker,
                         idx_t* nbrs);
void CreateGraphNodal(idx_t ne, idx_t nn, idx_t* eptr, idx_t* eind, idx_t** r_xadj, idx_t** r_adjncy);
idx_t FindCommonNodes(
    idx_t qid, idx_t nelmnts, idx_t* elmntids, idx_t* eptr, idx_t* eind, idx_t* marker, idx_t* nbrs);
mesh_t* CreateMesh(void);
void    InitMesh(mesh_t* mesh);
void    FreeMesh(mesh_t** mesh);


/* meshpart.c */
void InduceRowPartFromColumnPart(idx_t   nrows,
                                 idx_t*  rowptr,
                                 idx_t*  rowind,
                                 idx_t*  rpart,
                                 idx_t*  cpart,
                                 idx_t   nparts,
                                 real_t* tpwgts);


/* minconn.c */
void ComputeSubDomainGraph(ctrl_t* ctrl, graph_t* graph);
void UpdateEdgeSubDomainGraph(ctrl_t* ctrl, idx_t u, idx_t v, idx_t ewgt, idx_t* r_maxndoms);
void PrintSubDomainGraph(graph_t* graph, idx_t nparts, idx_t* where);
void EliminateSubDomainEdges(ctrl_t* ctrl, graph_t* graph);
void MoveGroupMinConnForCut(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t nind, idx_t* ind);
void MoveGroupMinConnForVol(ctrl_t*  ctrl,
                            graph_t* graph,
                            idx_t    to,
                            idx_t    nind,
                            idx_t*   ind,
                            idx_t*   vmarker,
                            idx_t*   pmarker,
                            idx_t*   modind);


/* mincover.o */
void  MinCover(idx_t*, idx_t*, idx_t, idx_t, idx_t*, idx_t*);
idx_t MinCover_Augment(idx_t*, idx_t*, idx_t, idx_t*, idx_t*, idx_t*, idx_t);
void  MinCover_Decompose(idx_t*, idx_t*, idx_t, idx_t, idx_t*, idx_t*, idx_t*);
void  MinCover_ColDFS(idx_t*, idx_t*, idx_t, idx_t*, idx_t*, idx_t);
void  MinCover_RowDFS(idx_t*, idx_t*, idx_t, idx_t*, idx_t*, idx_t);


/* mmd.c */
void genmmd(idx_t, idx_t*, idx_t*, idx_t*, idx_t*, idx_t, idx_t*, idx_t*, idx_t*, idx_t*, idx_t, idx_t*);
void mmdelm(idx_t, idx_t* xadj, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t, idx_t);
idx_t mmdint(idx_t, idx_t* xadj, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*);
void mmdnum(idx_t, idx_t*, idx_t*, idx_t*);
void mmdupd(idx_t, idx_t, idx_t*, idx_t*, idx_t, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t*, idx_t, idx_t* tag);


/* ometis.c */
void MlevelNestedDissection(ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx);
void MlevelNestedDissectionCC(ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx);
void MlevelNodeBisectionMultiple(ctrl_t* ctrl, graph_t* graph);
void MlevelNodeBisectionL2(ctrl_t* ctrl, graph_t* graph, idx_t niparts);
void MlevelNodeBisectionL1(ctrl_t* ctrl, graph_t* graph, idx_t niparts);
void SplitGraphOrder(ctrl_t* ctrl, graph_t* graph, graph_t** r_lgraph, graph_t** r_rgraph);
graph_t** SplitGraphOrderCC(ctrl_t* ctrl, graph_t* graph, idx_t ncmps, idx_t* cptr, idx_t* cind);
void MMDOrder(ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx);


/* options.c */
ctrl_t* SetupCtrl(moptype_et optype, idx_t* options, idx_t ncon, idx_t nparts, real_t* tpwgts, real_t* ubvec);
void SetupKWayBalMultipliers(ctrl_t* ctrl, graph_t* graph);
void Setup2WayBalMultipliers(ctrl_t* ctrl, graph_t* graph, real_t* tpwgts);
void PrintCtrl(ctrl_t* ctrl);
int  CheckParams(ctrl_t* ctrl);
void FreeCtrl(ctrl_t** r_ctrl);


/* parmetis.c */
void MlevelNestedDissectionP(
    ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx, idx_t npes, idx_t cpos, idx_t* sizes);
void FM_2WayNodeRefine1SidedP(ctrl_t* ctrl, graph_t* graph, idx_t* hmarker, real_t ubfactor, idx_t npasses);
void FM_2WayNodeRefine2SidedP(ctrl_t* ctrl, graph_t* graph, idx_t* hmarker, real_t ubfactor, idx_t npasses);


/* pmetis.c */
idx_t MlevelRecursiveBisection(
    ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* part, real_t* tpwgts, idx_t fpart);
idx_t MultilevelBisect(ctrl_t* ctrl, graph_t* graph, real_t* tpwgts);
void SplitGraphPart(ctrl_t* ctrl, graph_t* graph, graph_t** r_lgraph, graph_t** r_rgraph);


/* refine.c */
void Refine2Way(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph, real_t* rtpwgts);
void Allocate2WayPartitionMemory(ctrl_t* ctrl, graph_t* graph);
void Compute2WayPartitionParams(ctrl_t* ctrl, graph_t* graph);
void Project2WayPartition(ctrl_t* ctrl, graph_t* graph);


/* separator.c */
void ConstructSeparator(ctrl_t* ctrl, graph_t* graph);
void ConstructMinCoverSeparator(ctrl_t* ctrl, graph_t* graph);


/* sfm.c */
void FM_2WayNodeRefine2Sided(ctrl_t* ctrl, graph_t* graph, idx_t niter);
void FM_2WayNodeRefine1Sided(ctrl_t* ctrl, graph_t* graph, idx_t niter);
void FM_2WayNodeBalance(ctrl_t* ctrl, graph_t* graph);


/* srefine.c */
void Refine2WayNode(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph);
void Allocate2WayNodePartitionMemory(ctrl_t* ctrl, graph_t* graph);
void Compute2WayNodePartitionParams(ctrl_t* ctrl, graph_t* graph);
void Project2WayNodePartition(ctrl_t* ctrl, graph_t* graph);


/* stat.c */
void   ComputePartitionInfoBipartite(graph_t*, idx_t, idx_t*);
void   ComputePartitionBalance(graph_t*, idx_t, idx_t*, real_t*);
real_t ComputeElementBalance(idx_t, idx_t, idx_t*);


/* timing.c */
void InitTimers(ctrl_t*);
void PrintTimers(ctrl_t*);

/* util.c */
idx_t iargmax_strd(size_t, idx_t*, idx_t);
idx_t iargmax_nrm(size_t n, idx_t* x, real_t* y);
idx_t iargmax2_nrm(size_t n, idx_t* x, real_t* y);
idx_t rargmax2(size_t, real_t*);
void  InitRandom(idx_t);
int   metis_rcode(int sigrval);


/* wspace.c */
void AllocateWorkSpace(ctrl_t* ctrl, graph_t* graph);
void AllocateRefinementWorkSpace(ctrl_t* ctrl, idx_t nbrpoolsize_max, idx_t nbrpoolsize);
void    FreeWorkSpace(ctrl_t* ctrl);
void*   wspacemalloc(ctrl_t* ctrl, size_t nbytes);
void    wspacepush(ctrl_t* ctrl);
void    wspacepop(ctrl_t* ctrl);
idx_t*  iwspacemalloc(ctrl_t*, idx_t);
real_t* rwspacemalloc(ctrl_t*, idx_t);
ikv_t*  ikvwspacemalloc(ctrl_t*, idx_t);
void    cnbrpoolReset(ctrl_t* ctrl);
idx_t   cnbrpoolGetNext(ctrl_t* ctrl, idx_t nnbrs);
void    vnbrpoolReset(ctrl_t* ctrl);
idx_t   vnbrpoolGetNext(ctrl_t* ctrl, idx_t nnbrs);


#endif

/* Merged internal METIS headers */

/* Merged internal METIS headers */
#endif /* _METIS_H_ */
