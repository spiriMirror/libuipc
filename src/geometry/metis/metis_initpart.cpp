/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 * This file is split from the port's merged implementation.
 */

#include <metis.h>

/* The merged implementation removes unused source files and GK_MK* macro
 * instantiations while preserving the METIS_PartGraphKway algorithm.
 */

/************************ initpart.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * initpart.c
 *
 * This file contains code that performs the initial partition of the
 * coarsest graph
 *
 * Started 7/23/97
 * George
 *
 */


/*************************************************************************/
/*! This function computes the initial bisection of the coarsest graph */
/*************************************************************************/
void Init2WayPartition(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    mdbglvl_et dbglvl;

    ASSERT(graph->tvwgt[0] >= 0);

    dbglvl = (mdbglvl_et)ctrl->dbglvl;
    IFSET(ctrl->dbglvl, METIS_DBG_REFINE, ctrl->dbglvl -= METIS_DBG_REFINE);
    IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, ctrl->dbglvl -= METIS_DBG_MOVEINFO);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->InitPartTmr));

    switch(ctrl->iptype)
    {
        case METIS_IPTYPE_RANDOM:
            if(graph->ncon == 1)
                RandomBisection(ctrl, graph, ntpwgts, niparts);
            else
                McRandomBisection(ctrl, graph, ntpwgts, niparts);
            break;

        case METIS_IPTYPE_GROW:
            if(graph->nedges == 0)
                if(graph->ncon == 1)
                    RandomBisection(ctrl, graph, ntpwgts, niparts);
                else
                    McRandomBisection(ctrl, graph, ntpwgts, niparts);
            else if(graph->ncon == 1)
                GrowBisection(ctrl, graph, ntpwgts, niparts);
            else
                McGrowBisection(ctrl, graph, ntpwgts, niparts);
            break;

        default:
            gk_errexit(SIGERR, "Unknown initial partition type: %d\n", ctrl->iptype);
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_IPART,
          printf("Initial Cut: %" PRIDX "\n", graph->mincut));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->InitPartTmr));
    ctrl->dbglvl = (idx_t)dbglvl;
}


/*************************************************************************/
/*! This function computes the initial separator of the coarsest graph */
/*************************************************************************/
void InitSeparator(ctrl_t* ctrl, graph_t* graph, idx_t niparts)
{
    real_t     ntpwgts[2] = {0.5, 0.5};
    mdbglvl_et dbglvl;

    dbglvl = (mdbglvl_et)ctrl->dbglvl;
    IFSET(ctrl->dbglvl, METIS_DBG_REFINE, ctrl->dbglvl -= METIS_DBG_REFINE);
    IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, ctrl->dbglvl -= METIS_DBG_MOVEINFO);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->InitPartTmr));

    /* this is required for the cut-based part of the refinement */
    Setup2WayBalMultipliers(ctrl, graph, ntpwgts);

    switch(ctrl->iptype)
    {
        case METIS_IPTYPE_EDGE:
            if(graph->nedges == 0)
                RandomBisection(ctrl, graph, ntpwgts, niparts);
            else
                GrowBisection(ctrl, graph, ntpwgts, niparts);

            Compute2WayPartitionParams(ctrl, graph);
            ConstructSeparator(ctrl, graph);
            break;

        case METIS_IPTYPE_NODE:
            GrowBisectionNode(ctrl, graph, ntpwgts, niparts);
            break;

        default:
            gk_errexit(SIGERR, "Unknown iptype of %" PRIDX "\n", ctrl->iptype);
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_IPART,
          printf("Initial Sep: %" PRIDX "\n", graph->mincut));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->InitPartTmr));

    ctrl->dbglvl = dbglvl;
}


/*************************************************************************/
/*! This function computes a bisection of a graph by randomly assigning
    the vertices followed by a bisection refinement.
    The resulting partition is returned in graph->where.
*/
/*************************************************************************/
void RandomBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    idx_t i, ii, j, k, nvtxs, pwgts[2], zeromaxpwgt, from, me,
        bestcut = 0, icut, mincut, inbfs;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where;
    idx_t *perm, *bestwhere;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    Allocate2WayPartitionMemory(ctrl, graph);
    where = graph->where;

    bestwhere = iwspacemalloc(ctrl, nvtxs);
    perm      = iwspacemalloc(ctrl, nvtxs);

    zeromaxpwgt = ctrl->ubfactors[0] * graph->tvwgt[0] * ntpwgts[0];

    for(inbfs = 0; inbfs < niparts; inbfs++)
    {
        iset(nvtxs, 1, where);

        if(inbfs > 0)
        {
            irandArrayPermute(nvtxs, perm, nvtxs / 2, 1);
            pwgts[1] = graph->tvwgt[0];
            pwgts[0] = 0;

            for(ii = 0; ii < nvtxs; ii++)
            {
                i = perm[ii];
                if(pwgts[0] + vwgt[i] < zeromaxpwgt)
                {
                    where[i] = 0;
                    pwgts[0] += vwgt[i];
                    pwgts[1] -= vwgt[i];
                    if(pwgts[0] > zeromaxpwgt)
                        break;
                }
            }
        }

        /* Do some partition refinement  */
        Compute2WayPartitionParams(ctrl, graph);
        /* printf("IPART: %3"  PRIDX  " [%5"  PRIDX  " %5"  PRIDX  "] [%5"  PRIDX  " %5"  PRIDX  "] %5"  PRIDX  "\n", graph->nvtxs, pwgts[0], pwgts[1], graph->pwgts[0], graph->pwgts[1], graph->mincut); */

        Balance2Way(ctrl, graph, ntpwgts);
        /* printf("BPART: [%5"  PRIDX  " %5"  PRIDX  "] %5"  PRIDX  "\n", graph->pwgts[0], graph->pwgts[1], graph->mincut); */

        FM_2WayRefine(ctrl, graph, ntpwgts, 4);
        /* printf("RPART: [%5"  PRIDX  " %5"  PRIDX  "] %5"  PRIDX  "\n", graph->pwgts[0], graph->pwgts[1], graph->mincut); */

        if(inbfs == 0 || bestcut > graph->mincut)
        {
            bestcut = graph->mincut;
            icopy(nvtxs, where, bestwhere);
            if(bestcut == 0)
                break;
        }
    }

    graph->mincut = bestcut;
    icopy(nvtxs, bestwhere, where);

    WCOREPOP;
}


/*************************************************************************/
/*! This function takes a graph and produces a bisection by using a region
    growing algorithm. The resulting bisection is refined using FM.
    The resulting partition is returned in graph->where.
*/
/*************************************************************************/
void GrowBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    idx_t i, j, k, nvtxs, drain, nleft, first, last, pwgts[2], oneminpwgt,
        onemaxpwgt, from, me, bestcut = 0, icut, mincut, inbfs;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where;
    idx_t *queue, *touched, *gain, *bestwhere;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    Allocate2WayPartitionMemory(ctrl, graph);
    where = graph->where;

    bestwhere = iwspacemalloc(ctrl, nvtxs);
    queue     = iwspacemalloc(ctrl, nvtxs);
    touched   = iwspacemalloc(ctrl, nvtxs);

    onemaxpwgt = ctrl->ubfactors[0] * graph->tvwgt[0] * ntpwgts[1];
    oneminpwgt = (1.0 / ctrl->ubfactors[0]) * graph->tvwgt[0] * ntpwgts[1];

    for(inbfs = 0; inbfs < niparts; inbfs++)
    {
        iset(nvtxs, 1, where);

        iset(nvtxs, 0, touched);

        pwgts[1] = graph->tvwgt[0];
        pwgts[0] = 0;


        queue[0]          = irandInRange(nvtxs);
        touched[queue[0]] = 1;
        first             = 0;
        last              = 1;
        nleft             = nvtxs - 1;
        drain             = 0;

        /* Start the BFS from queue to get a partition */
        for(;;)
        {
            if(first == last)
            { /* Empty. Disconnected graph! */
                if(nleft == 0 || drain)
                    break;

                k = irandInRange(nleft);
                for(i = 0; i < nvtxs; i++)
                {
                    if(touched[i] == 0)
                    {
                        if(k == 0)
                            break;
                        else
                            k--;
                    }
                }

                queue[0]   = i;
                touched[i] = 1;
                first      = 0;
                last       = 1;
                nleft--;
            }

            i = queue[first++];
            if(pwgts[0] > 0 && pwgts[1] - vwgt[i] < oneminpwgt)
            {
                drain = 1;
                continue;
            }

            where[i] = 0;
            INC_DEC(pwgts[0], pwgts[1], vwgt[i]);
            if(pwgts[1] <= onemaxpwgt)
                break;

            drain = 0;
            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                k = adjncy[j];
                if(touched[k] == 0)
                {
                    queue[last++] = k;
                    touched[k]    = 1;
                    nleft--;
                }
            }
        }

        /* Check to see if we hit any bad limiting cases */
        if(pwgts[1] == 0)
            where[irandInRange(nvtxs)] = 1;
        if(pwgts[0] == 0)
            where[irandInRange(nvtxs)] = 0;

        /*************************************************************
    * Do some partition refinement
    **************************************************************/
        Compute2WayPartitionParams(ctrl, graph);
        /*
    printf("IPART: %3"  PRIDX  " [%5"  PRIDX  " %5"  PRIDX  "] [%5"  PRIDX  " %5"  PRIDX  "] %5"  PRIDX  "\n",
        graph->nvtxs, pwgts[0], pwgts[1], graph->pwgts[0], graph->pwgts[1], graph->mincut);
    */

        Balance2Way(ctrl, graph, ntpwgts);
        /*
    printf("BPART: [%5"  PRIDX  " %5"  PRIDX  "] %5"  PRIDX  "\n", graph->pwgts[0],
        graph->pwgts[1], graph->mincut);
    */

        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
        /*
    printf("RPART: [%5"  PRIDX  " %5"  PRIDX  "] %5"  PRIDX  "\n", graph->pwgts[0],
        graph->pwgts[1], graph->mincut);
    */

        if(inbfs == 0 || bestcut > graph->mincut)
        {
            bestcut = graph->mincut;
            icopy(nvtxs, where, bestwhere);
            if(bestcut == 0)
                break;
        }
    }

    graph->mincut = bestcut;
    icopy(nvtxs, bestwhere, where);

    WCOREPOP;
}


/*************************************************************************/
/*! This function takes a multi-constraint graph and computes a bisection
    by randomly assigning the vertices and then refining it. The resulting
    partition is returned in graph->where.
*/
/**************************************************************************/
void McRandomBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    idx_t  i, ii, j, k, nvtxs, ncon, from, bestcut = 0, mincut, inbfs, qnum;
    idx_t *bestwhere, *where, *perm, *counts;
    idx_t* vwgt;

    WCOREPUSH;

    nvtxs = graph->nvtxs;
    ncon  = graph->ncon;
    vwgt  = graph->vwgt;

    Allocate2WayPartitionMemory(ctrl, graph);
    where = graph->where;

    bestwhere = iwspacemalloc(ctrl, nvtxs);
    perm      = iwspacemalloc(ctrl, nvtxs);
    counts    = iwspacemalloc(ctrl, ncon);

    for(inbfs = 0; inbfs < 2 * niparts; inbfs++)
    {
        irandArrayPermute(nvtxs, perm, nvtxs / 2, 1);
        iset(ncon, 0, counts);

        /* partition by splitting the queues randomly */
        for(ii = 0; ii < nvtxs; ii++)
        {
            i        = perm[ii];
            qnum     = iargmax(ncon, vwgt + i * ncon, 1);
            where[i] = (counts[qnum]++) % 2;
        }

        Compute2WayPartitionParams(ctrl, graph);

        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
        Balance2Way(ctrl, graph, ntpwgts);
        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
        Balance2Way(ctrl, graph, ntpwgts);
        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

        if(inbfs == 0 || bestcut >= graph->mincut)
        {
            bestcut = graph->mincut;
            icopy(nvtxs, where, bestwhere);
            if(bestcut == 0)
                break;
        }
    }

    graph->mincut = bestcut;
    icopy(nvtxs, bestwhere, where);

    WCOREPOP;
}


/*************************************************************************/
/*! This function takes a multi-constraint graph and produces a bisection
    by using a region growing algorithm. The resulting partition is
    returned in graph->where.
*/
/*************************************************************************/
void McGrowBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    idx_t  i, j, k, nvtxs, ncon, from, bestcut = 0, mincut, inbfs;
    idx_t *bestwhere, *where;

    WCOREPUSH;

    nvtxs = graph->nvtxs;

    Allocate2WayPartitionMemory(ctrl, graph);
    where = graph->where;

    bestwhere = iwspacemalloc(ctrl, nvtxs);

    for(inbfs = 0; inbfs < 2 * niparts; inbfs++)
    {
        iset(nvtxs, 1, where);
        where[irandInRange(nvtxs)] = 0;

        Compute2WayPartitionParams(ctrl, graph);

        Balance2Way(ctrl, graph, ntpwgts);
        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
        Balance2Way(ctrl, graph, ntpwgts);
        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

        if(inbfs == 0 || bestcut >= graph->mincut)
        {
            bestcut = graph->mincut;
            icopy(nvtxs, where, bestwhere);
            if(bestcut == 0)
                break;
        }
    }

    graph->mincut = bestcut;
    icopy(nvtxs, bestwhere, where);

    WCOREPOP;
}


/*************************************************************************/
/* This function takes a graph and produces a tri-section into left, right,
   and separator using a region growing algorithm. The resulting separator
   is refined using node FM.
   The resulting partition is returned in graph->where.
*/
/**************************************************************************/
void GrowBisectionNode(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    idx_t i, j, k, nvtxs, drain, nleft, first, last, pwgts[2], oneminpwgt,
        onemaxpwgt, from, me, bestcut = 0, icut, mincut, inbfs;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *bndind;
    idx_t *queue, *touched, *gain, *bestwhere;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    bestwhere = iwspacemalloc(ctrl, nvtxs);
    queue     = iwspacemalloc(ctrl, nvtxs);
    touched   = iwspacemalloc(ctrl, nvtxs);

    onemaxpwgt = ctrl->ubfactors[0] * graph->tvwgt[0] * 0.5;
    oneminpwgt = (1.0 / ctrl->ubfactors[0]) * graph->tvwgt[0] * 0.5;


    /* Allocate refinement memory. Allocate sufficient memory for both edge and node */
    graph->pwgts  = imalloc(3, "GrowBisectionNode: pwgts");
    graph->where  = imalloc(nvtxs, "GrowBisectionNode: where");
    graph->bndptr = imalloc(nvtxs, "GrowBisectionNode: bndptr");
    graph->bndind = imalloc(nvtxs, "GrowBisectionNode: bndind");
    graph->id     = imalloc(nvtxs, "GrowBisectionNode: id");
    graph->ed     = imalloc(nvtxs, "GrowBisectionNode: ed");
    graph->nrinfo = (nrinfo_t*)gk_malloc(nvtxs * sizeof(nrinfo_t), "GrowBisectionNode: nrinfo");

    where  = graph->where;
    bndind = graph->bndind;

    for(inbfs = 0; inbfs < niparts; inbfs++)
    {
        iset(nvtxs, 1, where);
        iset(nvtxs, 0, touched);

        pwgts[1] = graph->tvwgt[0];
        pwgts[0] = 0;

        queue[0]          = irandInRange(nvtxs);
        touched[queue[0]] = 1;
        first             = 0;
        last              = 1;
        nleft             = nvtxs - 1;
        drain             = 0;

        /* Start the BFS from queue to get a partition */
        for(;;)
        {
            if(first == last)
            { /* Empty. Disconnected graph! */
                if(nleft == 0 || drain)
                    break;

                k = irandInRange(nleft);
                for(i = 0; i < nvtxs; i++)
                { /* select the kth untouched vertex */
                    if(touched[i] == 0)
                    {
                        if(k == 0)
                            break;
                        else
                            k--;
                    }
                }

                queue[0]   = i;
                touched[i] = 1;
                first      = 0;
                last       = 1;
                nleft--;
            }

            i = queue[first++];
            if(pwgts[1] - vwgt[i] < oneminpwgt)
            {
                drain = 1;
                continue;
            }

            where[i] = 0;
            INC_DEC(pwgts[0], pwgts[1], vwgt[i]);
            if(pwgts[1] <= onemaxpwgt)
                break;

            drain = 0;
            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                k = adjncy[j];
                if(touched[k] == 0)
                {
                    queue[last++] = k;
                    touched[k]    = 1;
                    nleft--;
                }
            }
        }

        /*************************************************************
    * Do some partition refinement
    **************************************************************/
        Compute2WayPartitionParams(ctrl, graph);
        Balance2Way(ctrl, graph, ntpwgts);
        FM_2WayRefine(ctrl, graph, ntpwgts, 4);

        /* Construct and refine the vertex separator */
        for(i = 0; i < graph->nbnd; i++)
        {
            j = bndind[i];
            if(xadj[j + 1] - xadj[j] > 0) /* ignore islands */
                where[j] = 2;
        }

        Compute2WayNodePartitionParams(ctrl, graph);
        FM_2WayNodeRefine2Sided(ctrl, graph, 1);
        FM_2WayNodeRefine1Sided(ctrl, graph, 4);

        /*
    printf("ISep: [%"  PRIDX  " %"  PRIDX  " %"  PRIDX  " %"  PRIDX  "] %"  PRIDX  "\n",
        inbfs, graph->pwgts[0], graph->pwgts[1], graph->pwgts[2], bestcut);
    */

        if(inbfs == 0 || bestcut > graph->mincut)
        {
            bestcut = graph->mincut;
            icopy(nvtxs, where, bestwhere);
        }
    }

    graph->mincut = bestcut;
    icopy(nvtxs, bestwhere, where);

    WCOREPOP;
}


/*************************************************************************/
/* This function takes a graph and produces a tri-section into left, right,
   and separator using a region growing algorithm. The resulting separator
   is refined using node FM.
   The resulting partition is returned in graph->where.
*/
/**************************************************************************/
void GrowBisectionNode2(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niparts)
{
    idx_t  i, j, k, nvtxs, bestcut = 0, mincut, inbfs;
    idx_t *xadj, *where, *bndind, *bestwhere;

    WCOREPUSH;

    nvtxs = graph->nvtxs;
    xadj  = graph->xadj;

    /* Allocate refinement memory. Allocate sufficient memory for both edge and node */
    graph->pwgts  = imalloc(3, "GrowBisectionNode: pwgts");
    graph->where  = imalloc(nvtxs, "GrowBisectionNode: where");
    graph->bndptr = imalloc(nvtxs, "GrowBisectionNode: bndptr");
    graph->bndind = imalloc(nvtxs, "GrowBisectionNode: bndind");
    graph->id     = imalloc(nvtxs, "GrowBisectionNode: id");
    graph->ed     = imalloc(nvtxs, "GrowBisectionNode: ed");
    graph->nrinfo = (nrinfo_t*)gk_malloc(nvtxs * sizeof(nrinfo_t), "GrowBisectionNode: nrinfo");

    bestwhere = iwspacemalloc(ctrl, nvtxs);

    where  = graph->where;
    bndind = graph->bndind;

    for(inbfs = 0; inbfs < niparts; inbfs++)
    {
        iset(nvtxs, 1, where);
        if(inbfs > 0)
            where[irandInRange(nvtxs)] = 0;

        Compute2WayPartitionParams(ctrl, graph);
        General2WayBalance(ctrl, graph, ntpwgts);
        FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

        /* Construct and refine the vertex separator */
        for(i = 0; i < graph->nbnd; i++)
        {
            j = bndind[i];
            if(xadj[j + 1] - xadj[j] > 0) /* ignore islands */
                where[j] = 2;
        }

        Compute2WayNodePartitionParams(ctrl, graph);
        FM_2WayNodeRefine2Sided(ctrl, graph, 4);

        /*
    printf("ISep: [%"  PRIDX  " %"  PRIDX  " %"  PRIDX  " %"  PRIDX  "] %"  PRIDX  "\n",
        inbfs, graph->pwgts[0], graph->pwgts[1], graph->pwgts[2], bestcut);
    */

        if(inbfs == 0 || bestcut > graph->mincut)
        {
            bestcut = graph->mincut;
            icopy(nvtxs, where, bestwhere);
        }
    }

    graph->mincut = bestcut;
    icopy(nvtxs, bestwhere, where);

    WCOREPOP;
}

/************************ refine.c ************************/
/*
\file
\brief This file contains the driving routines for multilevel refinement

\date   Started 7/24/1997
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version\verbatim $Id: refine.c 14362 2013-05-21 21:35:23Z karypis $ \endverbatim
*/


/*************************************************************************/
/*! This function is the entry point of refinement */
/*************************************************************************/
void Refine2Way(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph, real_t* tpwgts)
{

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->UncoarsenTmr));

    /* Compute the parameters of the coarsest graph */
    Compute2WayPartitionParams(ctrl, graph);

    for(;;)
    {
        ASSERT(CheckBnd(graph));

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->RefTmr));

        Balance2Way(ctrl, graph, tpwgts);

        FM_2WayRefine(ctrl, graph, tpwgts, ctrl->niter);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->RefTmr));

        if(graph == orggraph)
            break;

        graph = graph->finer;
        graph_ReadFromDisk(ctrl, graph);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ProjectTmr));
        Project2WayPartition(ctrl, graph);
        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ProjectTmr));
    }

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->UncoarsenTmr));
}


/*************************************************************************/
/*! This function allocates memory for 2-way edge refinement */
/*************************************************************************/
void Allocate2WayPartitionMemory(ctrl_t* ctrl, graph_t* graph)
{
    idx_t nvtxs, ncon;

    nvtxs = graph->nvtxs;
    ncon  = graph->ncon;

    graph->pwgts  = imalloc(2 * ncon, "Allocate2WayPartitionMemory: pwgts");
    graph->where  = imalloc(nvtxs, "Allocate2WayPartitionMemory: where");
    graph->bndptr = imalloc(nvtxs, "Allocate2WayPartitionMemory: bndptr");
    graph->bndind = imalloc(nvtxs, "Allocate2WayPartitionMemory: bndind");
    graph->id     = imalloc(nvtxs, "Allocate2WayPartitionMemory: id");
    graph->ed     = imalloc(nvtxs, "Allocate2WayPartitionMemory: ed");
}


/*************************************************************************/
/*! This function computes the initial id/ed */
/*************************************************************************/
void Compute2WayPartitionParams(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, j, nvtxs, ncon, nbnd, mincut, istart, iend, tid, ted, me;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *pwgts;
    idx_t *where, *bndptr, *bndind, *id, *ed;

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    where = graph->where;
    id    = graph->id;
    ed    = graph->ed;

    pwgts  = iset(2 * ncon, 0, graph->pwgts);
    bndptr = iset(nvtxs, -1, graph->bndptr);
    bndind = graph->bndind;

    /* Compute pwgts */
    if(ncon == 1)
    {
        for(i = 0; i < nvtxs; i++)
        {
            ASSERT(where[i] >= 0 && where[i] <= 1);
            pwgts[where[i]] += vwgt[i];
        }
        ASSERT(pwgts[0] + pwgts[1] == graph->tvwgt[0]);
    }
    else
    {
        for(i = 0; i < nvtxs; i++)
        {
            me = where[i];
            for(j = 0; j < ncon; j++)
                pwgts[me * ncon + j] += vwgt[i * ncon + j];
        }
    }


    /* Compute the required info for refinement  */
    for(nbnd = 0, mincut = 0, i = 0; i < nvtxs; i++)
    {
        istart = xadj[i];
        iend   = xadj[i + 1];

        me  = where[i];
        tid = ted = 0;

        for(j = istart; j < iend; j++)
        {
            if(me == where[adjncy[j]])
                tid += adjwgt[j];
            else
                ted += adjwgt[j];
        }
        id[i] = tid;
        ed[i] = ted;

        if(ted > 0 || istart == iend)
        {
            BNDInsert(nbnd, bndind, bndptr, i);
            mincut += ted;
        }
    }

    graph->mincut = mincut / 2;
    graph->nbnd   = nbnd;
}


/*************************************************************************/
/*! Projects a partition and computes the refinement params. */
/*************************************************************************/
void Project2WayPartition(ctrl_t* ctrl, graph_t* graph)
{
    idx_t    i, j, istart, iend, nvtxs, nbnd, me, tid, ted;
    idx_t *  xadj, *adjncy, *adjwgt;
    idx_t *  cmap, *where, *bndptr, *bndind;
    idx_t *  cwhere, *cbndptr;
    idx_t *  id, *ed;
    graph_t* cgraph;
    int      dropedges;

    Allocate2WayPartitionMemory(ctrl, graph);

    dropedges = ctrl->dropedges;

    cgraph  = graph->coarser;
    cwhere  = cgraph->where;
    cbndptr = cgraph->bndptr;

    nvtxs  = graph->nvtxs;
    cmap   = graph->cmap;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    where = graph->where;
    id    = graph->id;
    ed    = graph->ed;

    bndptr = iset(nvtxs, -1, graph->bndptr);
    bndind = graph->bndind;

    /* Project the partition and record which of these nodes came from the
     coarser boundary */
    for(i = 0; i < nvtxs; i++)
    {
        j        = cmap[i];
        where[i] = cwhere[j];
        cmap[i]  = (dropedges ? 0 : cbndptr[j]);
    }

    /* Compute the refinement information of the nodes */
    for(nbnd = 0, i = 0; i < nvtxs; i++)
    {
        istart = xadj[i];
        iend   = xadj[i + 1];

        tid = ted = 0;
        if(cmap[i] == -1)
        { /* Interior node. Note that cmap[i] = cbndptr[cmap[i]] */
            for(j = istart; j < iend; j++)
                tid += adjwgt[j];
        }
        else
        { /* Potentially an interface node */
            me = where[i];
            for(j = istart; j < iend; j++)
            {
                if(me == where[adjncy[j]])
                    tid += adjwgt[j];
                else
                    ted += adjwgt[j];
            }
        }
        id[i] = tid;
        ed[i] = ted;

        if(ted > 0 || istart == iend)
            BNDInsert(nbnd, bndind, bndptr, i);
    }
    graph->mincut = (dropedges ? ComputeCut(graph, where) : cgraph->mincut);
    graph->nbnd   = nbnd;

    /* copy pwgts */
    icopy(2 * graph->ncon, cgraph->pwgts, graph->pwgts);

    FreeGraph(&graph->coarser);
    graph->coarser = NULL;
}

/************************ separator.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * separator.c
 *
 * This file contains code for separator extraction
 *
 * Started 8/1/97
 * George
 *
 * $Id: separator.c 10481 2011-07-05 18:01:23Z karypis $
 *
 */


/*************************************************************************
* This function takes a bisection and constructs a minimum weight vertex
* separator out of it. It uses the node-based separator refinement for it.
**************************************************************************/
void ConstructSeparator(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, j, k, nvtxs, nbnd;
    idx_t *xadj, *where, *bndind;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    nbnd   = graph->nbnd;
    bndind = graph->bndind;

    where = icopy(nvtxs, graph->where, iwspacemalloc(ctrl, nvtxs));

    /* Put the nodes in the boundary into the separator */
    for(i = 0; i < nbnd; i++)
    {
        j = bndind[i];
        if(xadj[j + 1] - xadj[j] > 0) /* Ignore islands */
            where[j] = 2;
    }

    FreeRData(graph);

    Allocate2WayNodePartitionMemory(ctrl, graph);
    icopy(nvtxs, where, graph->where);

    WCOREPOP;

    ASSERT(IsSeparable(graph));

    Compute2WayNodePartitionParams(ctrl, graph);

    ASSERT(CheckNodePartitionParams(graph));

    FM_2WayNodeRefine2Sided(ctrl, graph, 1);
    FM_2WayNodeRefine1Sided(ctrl, graph, 4);

    ASSERT(IsSeparable(graph));
}


/*************************************************************************
* This function takes a bisection and constructs a minimum weight vertex
* separator out of it. It uses an unweighted minimum-cover algorithm
* followed by node-based separator refinement.
**************************************************************************/
void ConstructMinCoverSeparator(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, ii, j, jj, k, l, nvtxs, nbnd, bnvtxs[3], bnedges[2], csize;
    idx_t *xadj, *adjncy, *bxadj, *badjncy;
    idx_t *where, *bndind, *bndptr, *vmap, *ivmap, *cover;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;

    nbnd   = graph->nbnd;
    bndind = graph->bndind;
    bndptr = graph->bndptr;
    where  = graph->where;

    vmap  = iwspacemalloc(ctrl, nvtxs);
    ivmap = iwspacemalloc(ctrl, nbnd);
    cover = iwspacemalloc(ctrl, nbnd);

    if(nbnd > 0)
    {
        /* Go through the boundary and determine the sizes of the bipartite graph */
        bnvtxs[0] = bnvtxs[1] = bnedges[0] = bnedges[1] = 0;
        for(i = 0; i < nbnd; i++)
        {
            j = bndind[i];
            k = where[j];
            if(xadj[j + 1] - xadj[j] > 0)
            {
                bnvtxs[k]++;
                bnedges[k] += xadj[j + 1] - xadj[j];
            }
        }

        bnvtxs[2] = bnvtxs[0] + bnvtxs[1];
        bnvtxs[1] = bnvtxs[0];
        bnvtxs[0] = 0;

        bxadj   = iwspacemalloc(ctrl, bnvtxs[2] + 1);
        badjncy = iwspacemalloc(ctrl, bnedges[0] + bnedges[1] + 1);

        /* Construct the ivmap and vmap */
        ASSERT(iset(nvtxs, -1, vmap) == vmap);
        for(i = 0; i < nbnd; i++)
        {
            j = bndind[i];
            k = where[j];
            if(xadj[j + 1] - xadj[j] > 0)
            {
                vmap[j]            = bnvtxs[k];
                ivmap[bnvtxs[k]++] = j;
            }
        }

        /* OK, go through and put the vertices of each part starting from 0 */
        bnvtxs[1] = bnvtxs[0];
        bnvtxs[0] = 0;
        bxadj[0] = l = 0;
        for(k = 0; k < 2; k++)
        {
            for(ii = 0; ii < nbnd; ii++)
            {
                i = bndind[ii];
                if(where[i] == k && xadj[i] < xadj[i + 1])
                {
                    for(j = xadj[i]; j < xadj[i + 1]; j++)
                    {
                        jj = adjncy[j];
                        if(where[jj] != k)
                        {
                            ASSERT(bndptr[jj] != -1);
                            ASSERTP(vmap[jj] != -1,
                                    ("%" PRIDX " %" PRIDX " %" PRIDX "\n",
                                     jj,
                                     vmap[jj],
                                     graph->bndptr[jj]));
                            badjncy[l++] = vmap[jj];
                        }
                    }
                    bxadj[++bnvtxs[k]] = l;
                }
            }
        }

        ASSERT(l <= bnedges[0] + bnedges[1]);

        MinCover(bxadj, badjncy, bnvtxs[0], bnvtxs[1], cover, &csize);

        IFSET(ctrl->dbglvl,
              METIS_DBG_SEPINFO,
              printf("Nvtxs: %6" PRIDX ", [%5" PRIDX " %5" PRIDX "], Cut: %6" PRIDX
                     ", SS: [%6" PRIDX " %6" PRIDX "], Cover: %6" PRIDX "\n",
                     nvtxs,
                     graph->pwgts[0],
                     graph->pwgts[1],
                     graph->mincut,
                     bnvtxs[0],
                     bnvtxs[1] - bnvtxs[0],
                     csize));

        for(i = 0; i < csize; i++)
        {
            j        = ivmap[cover[i]];
            where[j] = 2;
        }
    }
    else
    {
        IFSET(ctrl->dbglvl,
              METIS_DBG_SEPINFO,
              printf("Nvtxs: %6" PRIDX ", [%5" PRIDX " %5" PRIDX "], Cut: %6" PRIDX
                     ", SS: [%6" PRIDX " %6" PRIDX "], Cover: %6" PRIDX "\n",
                     nvtxs,
                     graph->pwgts[0],
                     graph->pwgts[1],
                     graph->mincut,
                     (idx_t)0,
                     (idx_t)0,
                     (idx_t)0));
    }

    /* Prepare to refine the vertex separator */
    icopy(nvtxs, graph->where, vmap);

    FreeRData(graph);

    Allocate2WayNodePartitionMemory(ctrl, graph);
    icopy(nvtxs, vmap, graph->where);

    WCOREPOP;

    Compute2WayNodePartitionParams(ctrl, graph);

    ASSERT(CheckNodePartitionParams(graph));

    FM_2WayNodeRefine1Sided(ctrl, graph, ctrl->niter);

    ASSERT(IsSeparable(graph));
}

/************************ srefine.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * srefine.c
 *
 * This file contains code for the separator refinement algorithms
 *
 * Started 8/1/97
 * George
 *
 * $Id: srefine.c 14362 2013-05-21 21:35:23Z karypis $
 *
 */


/*************************************************************************/
/*! This function is the entry point of the separator refinement.
    It does not perform any refinement on graph, but it starts by first
    projecting it to the next level finer graph and proceeds from there. */
/*************************************************************************/
void Refine2WayNode(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph)
{

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->UncoarsenTmr));

    if(graph == orggraph)
    {
        Compute2WayNodePartitionParams(ctrl, graph);
    }
    else
    {
        do
        {
            graph = graph->finer;

            graph_ReadFromDisk(ctrl, graph);

            IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ProjectTmr));
            Project2WayNodePartition(ctrl, graph);
            IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ProjectTmr));

            IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->RefTmr));
            FM_2WayNodeBalance(ctrl, graph);

            ASSERT(CheckNodePartitionParams(graph));

            switch(ctrl->rtype)
            {
                case METIS_RTYPE_SEP2SIDED:
                    FM_2WayNodeRefine2Sided(ctrl, graph, ctrl->niter);
                    break;
                case METIS_RTYPE_SEP1SIDED:
                    FM_2WayNodeRefine1Sided(ctrl, graph, ctrl->niter);
                    break;
                default:
                    gk_errexit(SIGERR, "Unknown rtype of %d\n", ctrl->rtype);
            }
            IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->RefTmr));

        } while(graph != orggraph);
    }

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->UncoarsenTmr));
}


/*************************************************************************/
/*! This function allocates memory for 2-way node-based refinement */
/**************************************************************************/
void Allocate2WayNodePartitionMemory(ctrl_t* ctrl, graph_t* graph)
{
    idx_t nvtxs;

    nvtxs = graph->nvtxs;

    graph->pwgts  = imalloc(3, "Allocate2WayNodePartitionMemory: pwgts");
    graph->where  = imalloc(nvtxs, "Allocate2WayNodePartitionMemory: where");
    graph->bndptr = imalloc(nvtxs, "Allocate2WayNodePartitionMemory: bndptr");
    graph->bndind = imalloc(nvtxs, "Allocate2WayNodePartitionMemory: bndind");
    graph->nrinfo = (nrinfo_t*)gk_malloc(nvtxs * sizeof(nrinfo_t),
                                         "Allocate2WayNodePartitionMemory: nrinfo");
}


/*************************************************************************/
/*! This function computes the edegrees[] to the left & right sides */
/*************************************************************************/
void Compute2WayNodePartitionParams(ctrl_t* ctrl, graph_t* graph)
{
    idx_t     i, j, nvtxs, nbnd;
    idx_t *   xadj, *adjncy, *vwgt;
    idx_t *   where, *pwgts, *bndind, *bndptr, *edegrees;
    nrinfo_t* rinfo;
    idx_t     me, other;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;

    where  = graph->where;
    rinfo  = graph->nrinfo;
    pwgts  = iset(3, 0, graph->pwgts);
    bndind = graph->bndind;
    bndptr = iset(nvtxs, -1, graph->bndptr);


    /*------------------------------------------------------------
  / Compute now the separator external degrees
  /------------------------------------------------------------*/
    nbnd = 0;
    for(i = 0; i < nvtxs; i++)
    {
        me = where[i];
        pwgts[me] += vwgt[i];

        ASSERT(me >= 0 && me <= 2);

        if(me == 2)
        { /* If it is on the separator do some computations */
            BNDInsert(nbnd, bndind, bndptr, i);

            edegrees    = rinfo[i].edegrees;
            edegrees[0] = edegrees[1] = 0;

            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                other = where[adjncy[j]];
                if(other != 2)
                    edegrees[other] += vwgt[adjncy[j]];
            }
        }
    }

    ASSERT(CheckNodeBnd(graph, nbnd));

    graph->mincut = pwgts[2];
    graph->nbnd   = nbnd;
}


/*************************************************************************/
/*! This function projects the node-based bisection */
/*************************************************************************/
void Project2WayNodePartition(ctrl_t* ctrl, graph_t* graph)
{
    idx_t    i, j, nvtxs;
    idx_t *  cmap, *where, *cwhere;
    graph_t* cgraph;

    cgraph = graph->coarser;
    cwhere = cgraph->where;

    nvtxs = graph->nvtxs;
    cmap  = graph->cmap;

    Allocate2WayNodePartitionMemory(ctrl, graph);
    where = graph->where;

    /* Project the partition */
    for(i = 0; i < nvtxs; i++)
    {
        where[i] = cwhere[cmap[i]];
        ASSERTP(where[i] >= 0 && where[i] <= 2,
                ("%" PRIDX " %" PRIDX " %" PRIDX " %" PRIDX "\n", i, cmap[i], where[i], cwhere[cmap[i]]));
    }

    FreeGraph(&graph->coarser);
    graph->coarser = NULL;

    Compute2WayNodePartitionParams(ctrl, graph);
}
