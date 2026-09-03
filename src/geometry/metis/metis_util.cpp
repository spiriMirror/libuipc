/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 * This file is split from the port's merged implementation.
 */

#include <metis.h>

/* The merged implementation removes unused source files and GK_MK* macro
 * instantiations while preserving the METIS_PartGraphKway algorithm.
 */

/************************ balance.c ************************/
/*!
\file
\brief Functions for the edge-based balancing

\date Started 7/23/97
\author George
\author Copyright 1997-2011, Regents of the University of Minnesota
\version\verbatim $Id: balance.c 10187 2011-06-13 13:46:57Z karypis $ \endverbatim
*/


/*************************************************************************
* This function is the entry poidx_t of the bisection balancing algorithms.
**************************************************************************/
void Balance2Way(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
{
    if(ComputeLoadImbalanceDiff(graph, 2, ctrl->pijbm, ctrl->ubfactors) <= 0)
        return;

    if(graph->ncon == 1)
    {
        /* return right away if the balance is OK */
        if(rabs(ntpwgts[0] * graph->tvwgt[0] - graph->pwgts[0])
           < 3 * graph->tvwgt[0] / graph->nvtxs)
            return;

        if(graph->nbnd > 0)
            Bnd2WayBalance(ctrl, graph, ntpwgts);
        else
            General2WayBalance(ctrl, graph, ntpwgts);
    }
    else
    {
        McGeneral2WayBalance(ctrl, graph, ntpwgts);
    }
}


/*************************************************************************
* This function balances two partitions by moving boundary nodes
* from the domain that is overweight to the one that is underweight.
**************************************************************************/
void Bnd2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
{
    idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, me, tmp;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *id, *ed, *bndptr, *bndind, *pwgts;
    idx_t *moved, *perm;
    rpq_t* queue;
    idx_t  higain, mincut, mindiff;
    idx_t  tpwgts[2];

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    where  = graph->where;
    id     = graph->id;
    ed     = graph->ed;
    pwgts  = graph->pwgts;
    bndptr = graph->bndptr;
    bndind = graph->bndind;

    moved = iwspacemalloc(ctrl, nvtxs);
    perm  = iwspacemalloc(ctrl, nvtxs);

    /* Determine from which domain you will be moving data */
    tpwgts[0] = graph->tvwgt[0] * ntpwgts[0];
    tpwgts[1] = graph->tvwgt[0] - tpwgts[0];
    mindiff   = iabs(tpwgts[0] - pwgts[0]);
    from      = (pwgts[0] < tpwgts[0] ? 1 : 0);
    to        = (from + 1) % 2;

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("Partitions: [%6" PRIDX " %6" PRIDX "] T[%6" PRIDX " %6" PRIDX
                 "], Nv-Nb[%6" PRIDX " %6" PRIDX "]. ICut: %6" PRIDX " [B]\n",
                 pwgts[0],
                 pwgts[1],
                 tpwgts[0],
                 tpwgts[1],
                 graph->nvtxs,
                 graph->nbnd,
                 graph->mincut));

    queue = rpqCreate(nvtxs);

    iset(nvtxs, -1, moved);

    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERT(CheckBnd(graph));

    /* Insert the boundary nodes of the proper partition whose size is OK in the priority queue */
    nbnd = graph->nbnd;
    irandArrayPermute(nbnd, perm, nbnd / 5, 1);
    for(ii = 0; ii < nbnd; ii++)
    {
        i = perm[ii];
        ASSERT(ed[bndind[i]] > 0 || id[bndind[i]] == 0);
        ASSERT(bndptr[bndind[i]] != -1);
        if(where[bndind[i]] == from && vwgt[bndind[i]] <= mindiff)
            rpqInsert(queue, bndind[i], ed[bndind[i]] - id[bndind[i]]);
    }

    mincut = graph->mincut;
    for(nswaps = 0; nswaps < nvtxs; nswaps++)
    {
        if((higain = rpqGetTop(queue)) == -1)
            break;
        ASSERT(bndptr[higain] != -1);

        if(pwgts[to] + vwgt[higain] > tpwgts[to])
            break;

        mincut -= (ed[higain] - id[higain]);
        INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

        where[higain] = to;
        moved[higain] = nswaps;

        IFSET(ctrl->dbglvl,
              METIS_DBG_MOVEINFO,
              printf("Moved %6" PRIDX " from %" PRIDX ". [%3" PRIDX " %3" PRIDX
                     "] %5" PRIDX " [%4" PRIDX " %4" PRIDX "]\n",
                     higain,
                     from,
                     ed[higain] - id[higain],
                     vwgt[higain],
                     mincut,
                     pwgts[0],
                     pwgts[1]));

        /**************************************************************
    * Update the id[i]/ed[i] values of the affected nodes
    ***************************************************************/
        SWAP(id[higain], ed[higain], tmp);
        if(ed[higain] == 0 && xadj[higain] < xadj[higain + 1])
            BNDDelete(nbnd, bndind, bndptr, higain);

        for(j = xadj[higain]; j < xadj[higain + 1]; j++)
        {
            k    = adjncy[j];
            kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
            INC_DEC(id[k], ed[k], kwgt);

            /* Update its boundary information and queue position */
            if(bndptr[k] != -1)
            { /* If k was a boundary vertex */
                if(ed[k] == 0)
                { /* Not a boundary vertex any more */
                    BNDDelete(nbnd, bndind, bndptr, k);
                    if(moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff) /* Remove it if in the queues */
                        rpqDelete(queue, k);
                }
                else
                { /* If it has not been moved, update its position in the queue */
                    if(moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
                        rpqUpdate(queue, k, ed[k] - id[k]);
                }
            }
            else
            {
                if(ed[k] > 0)
                { /* It will now become a boundary vertex */
                    BNDInsert(nbnd, bndind, bndptr, k);
                    if(moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
                        rpqInsert(queue, k, ed[k] - id[k]);
                }
            }
        }
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("\tMinimum cut: %6" PRIDX ", PWGTS: [%6" PRIDX " %6" PRIDX
                 "], NBND: %6" PRIDX "\n",
                 mincut,
                 pwgts[0],
                 pwgts[1],
                 nbnd));

    graph->mincut = mincut;
    graph->nbnd   = nbnd;

    rpqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************
* This function balances two partitions by moving the highest gain
* (including negative gain) vertices to the other domain.
* It is used only when the unbalance is due to non contiguous
* subdomains. That is, the are no boundary vertices.
* It moves vertices from the domain that is overweight to the one that
* is underweight.
**************************************************************************/
void General2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
{
    idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, me, tmp;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *id, *ed, *bndptr, *bndind, *pwgts;
    idx_t *moved, *perm;
    rpq_t* queue;
    idx_t  higain, mincut, mindiff;
    idx_t  tpwgts[2];

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    where  = graph->where;
    id     = graph->id;
    ed     = graph->ed;
    pwgts  = graph->pwgts;
    bndptr = graph->bndptr;
    bndind = graph->bndind;

    moved = iwspacemalloc(ctrl, nvtxs);
    perm  = iwspacemalloc(ctrl, nvtxs);

    /* Determine from which domain you will be moving data */
    tpwgts[0] = graph->tvwgt[0] * ntpwgts[0];
    tpwgts[1] = graph->tvwgt[0] - tpwgts[0];
    mindiff   = iabs(tpwgts[0] - pwgts[0]);
    from      = (pwgts[0] < tpwgts[0] ? 1 : 0);
    to        = (from + 1) % 2;

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("Partitions: [%6" PRIDX " %6" PRIDX "] T[%6" PRIDX " %6" PRIDX
                 "], Nv-Nb[%6" PRIDX " %6" PRIDX "]. ICut: %6" PRIDX " [B]\n",
                 pwgts[0],
                 pwgts[1],
                 tpwgts[0],
                 tpwgts[1],
                 graph->nvtxs,
                 graph->nbnd,
                 graph->mincut));

    queue = rpqCreate(nvtxs);

    iset(nvtxs, -1, moved);

    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERT(CheckBnd(graph));

    /* Insert the nodes of the proper partition whose size is OK in the priority queue */
    irandArrayPermute(nvtxs, perm, nvtxs / 5, 1);
    for(ii = 0; ii < nvtxs; ii++)
    {
        i = perm[ii];
        if(where[i] == from && vwgt[i] <= mindiff)
            rpqInsert(queue, i, ed[i] - id[i]);
    }

    mincut = graph->mincut;
    nbnd   = graph->nbnd;
    for(nswaps = 0; nswaps < nvtxs; nswaps++)
    {
        if((higain = rpqGetTop(queue)) == -1)
            break;

        if(pwgts[to] + vwgt[higain] > tpwgts[to])
            break;

        mincut -= (ed[higain] - id[higain]);
        INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

        where[higain] = to;
        moved[higain] = nswaps;

        IFSET(ctrl->dbglvl,
              METIS_DBG_MOVEINFO,
              printf("Moved %6" PRIDX " from %" PRIDX ". [%3" PRIDX " %3" PRIDX
                     "] %5" PRIDX " [%4" PRIDX " %4" PRIDX "]\n",
                     higain,
                     from,
                     ed[higain] - id[higain],
                     vwgt[higain],
                     mincut,
                     pwgts[0],
                     pwgts[1]));

        /**************************************************************
    * Update the id[i]/ed[i] values of the affected nodes
    ***************************************************************/
        SWAP(id[higain], ed[higain], tmp);
        if(ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
            BNDDelete(nbnd, bndind, bndptr, higain);
        if(ed[higain] > 0 && bndptr[higain] == -1)
            BNDInsert(nbnd, bndind, bndptr, higain);

        for(j = xadj[higain]; j < xadj[higain + 1]; j++)
        {
            k = adjncy[j];

            kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
            INC_DEC(id[k], ed[k], kwgt);

            /* Update the queue position */
            if(moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
                rpqUpdate(queue, k, ed[k] - id[k]);

            /* Update its boundary information */
            if(ed[k] == 0 && bndptr[k] != -1)
                BNDDelete(nbnd, bndind, bndptr, k);
            else if(ed[k] > 0 && bndptr[k] == -1)
                BNDInsert(nbnd, bndind, bndptr, k);
        }
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("\tMinimum cut: %6" PRIDX ", PWGTS: [%6" PRIDX " %6" PRIDX
                 "], NBND: %6" PRIDX "\n",
                 mincut,
                 pwgts[0],
                 pwgts[1],
                 nbnd));

    graph->mincut = mincut;
    graph->nbnd   = nbnd;

    rpqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************
* This function performs an edge-based FM refinement
**************************************************************************/
void McGeneral2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
{
    idx_t i, ii, j, k, l, kwgt, nvtxs, ncon, nbnd, nswaps, from, to, pass, me,
        limit, tmp, cnum;
    idx_t *xadj, *adjncy, *vwgt, *adjwgt, *where, *pwgts, *id, *ed, *bndptr, *bndind;
    idx_t * moved, *swaps, *perm, *qnum, *qsizes;
    idx_t   higain, mincut, newcut, mincutorder;
    real_t *invtvwgt, *minbalv, *newbalv, minbal, newbal;
    rpq_t** queues;

    WCOREPUSH;

    nvtxs    = graph->nvtxs;
    ncon     = graph->ncon;
    xadj     = graph->xadj;
    vwgt     = graph->vwgt;
    adjncy   = graph->adjncy;
    adjwgt   = graph->adjwgt;
    invtvwgt = graph->invtvwgt;
    where    = graph->where;
    id       = graph->id;
    ed       = graph->ed;
    pwgts    = graph->pwgts;
    bndptr   = graph->bndptr;
    bndind   = graph->bndind;

    moved   = iwspacemalloc(ctrl, nvtxs);
    swaps   = iwspacemalloc(ctrl, nvtxs);
    perm    = iwspacemalloc(ctrl, nvtxs);
    qnum    = iwspacemalloc(ctrl, nvtxs);
    newbalv = rwspacemalloc(ctrl, ncon);
    minbalv = rwspacemalloc(ctrl, ncon);
    qsizes  = iwspacemalloc(ctrl, 2 * ncon);

    limit = gk_min(gk_max(0.01 * nvtxs, 15), 100);

    /* Initialize the queues */
    queues = (rpq_t**)wspacemalloc(ctrl, 2 * ncon * sizeof(rpq_t*));
    for(i = 0; i < 2 * ncon; i++)
    {
        queues[i] = rpqCreate(nvtxs);
        qsizes[i] = 0;
    }

    for(i = 0; i < nvtxs; i++)
    {
        qnum[i] = iargmax_nrm(ncon, vwgt + i * ncon, invtvwgt);
        qsizes[2 * qnum[i] + where[i]]++;
    }


    /* for the empty queues, move into them vertices from other queues */
    for(from = 0; from < 2; from++)
    {
        for(j = 0; j < ncon; j++)
        {
            if(qsizes[2 * j + from] == 0)
            {
                for(i = 0; i < nvtxs; i++)
                {
                    if(where[i] != from)
                        continue;

                    k = iargmax2_nrm(ncon, vwgt + i * ncon, invtvwgt);
                    if(k == j && qsizes[2 * qnum[i] + from] > qsizes[2 * j + from]
                       && vwgt[i * ncon + qnum[i]] * invtvwgt[qnum[i]]
                              < 1.3 * vwgt[i * ncon + j] * invtvwgt[j])
                    {
                        qsizes[2 * qnum[i] + from]--;
                        qsizes[2 * j + from]++;
                        qnum[i] = j;
                    }
                }
            }
        }
    }


    minbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ctrl->ubfactors, minbalv);
    ASSERT(minbal > 0.0);

    newcut = mincut = graph->mincut;
    mincutorder     = -1;

    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("Parts: [");
        for(l = 0; l < ncon; l++)
            printf("(%6" PRIDX " %6" PRIDX " %.3" PRREAL " %.3" PRREAL ") ",
                   pwgts[l],
                   pwgts[ncon + l],
                   ntpwgts[l],
                   ntpwgts[ncon + l]);
        printf("] Nv-Nb[%5" PRIDX ", %5" PRIDX "]. ICut: %6" PRIDX ", LB: %+.3" PRREAL " [B]\n",
               graph->nvtxs,
               graph->nbnd,
               graph->mincut,
               minbal);
    }

    iset(nvtxs, -1, moved);

    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERT(CheckBnd(graph));

    /* Insert all nodes in the priority queues */
    nbnd = graph->nbnd;
    irandArrayPermute(nvtxs, perm, nvtxs / 10, 1);
    for(ii = 0; ii < nvtxs; ii++)
    {
        i = perm[ii];
        rpqInsert(queues[2 * qnum[i] + where[i]], i, ed[i] - id[i]);
    }

    for(nswaps = 0; nswaps < nvtxs; nswaps++)
    {
        if(minbal <= 0.0)
            break;

        SelectQueue(graph, ctrl->pijbm, ctrl->ubfactors, queues, &from, &cnum);
        to = (from + 1) % 2;

        if(from == -1 || (higain = rpqGetTop(queues[2 * cnum + from])) == -1)
            break;

        newcut -= (ed[higain] - id[higain]);

        iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
        iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
        newbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ctrl->ubfactors, newbalv);

        if(newbal < minbal
           || (newbal == minbal
               && (newcut < mincut
                   || (newcut == mincut && BetterBalance2Way(ncon, minbalv, newbalv)))))
        {
            mincut      = newcut;
            minbal      = newbal;
            mincutorder = nswaps;
            rcopy(ncon, newbalv, minbalv);
        }
        else if(nswaps - mincutorder > limit)
        { /* We hit the limit, undo last move */
            newcut += (ed[higain] - id[higain]);
            iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
            iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
            break;
        }

        where[higain] = to;
        moved[higain] = nswaps;
        swaps[nswaps] = higain;

        if(ctrl->dbglvl & METIS_DBG_MOVEINFO)
        {
            printf("Moved %6" PRIDX " from %" PRIDX "(%" PRIDX "). Gain: %5" PRIDX
                   ", "
                   "Cut: %5" PRIDX ", NPwgts: ",
                   higain,
                   from,
                   cnum,
                   ed[higain] - id[higain],
                   newcut);
            for(l = 0; l < ncon; l++)
                printf("(%6" PRIDX ", %6" PRIDX ") ", pwgts[l], pwgts[ncon + l]);
            printf(", %+.3" PRREAL " LB: %+.3" PRREAL "\n", minbal, newbal);
        }


        /**************************************************************
    * Update the id[i]/ed[i] values of the affected nodes
    ***************************************************************/
        SWAP(id[higain], ed[higain], tmp);
        if(ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
            BNDDelete(nbnd, bndind, bndptr, higain);
        if(ed[higain] > 0 && bndptr[higain] == -1)
            BNDInsert(nbnd, bndind, bndptr, higain);

        for(j = xadj[higain]; j < xadj[higain + 1]; j++)
        {
            k = adjncy[j];

            kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
            INC_DEC(id[k], ed[k], kwgt);

            /* Update the queue position */
            if(moved[k] == -1)
                rpqUpdate(queues[2 * qnum[k] + where[k]], k, ed[k] - id[k]);

            /* Update its boundary information */
            if(ed[k] == 0 && bndptr[k] != -1)
                BNDDelete(nbnd, bndind, bndptr, k);
            else if(ed[k] > 0 && bndptr[k] == -1)
                BNDInsert(nbnd, bndind, bndptr, k);
        }
    }


    /****************************************************************
  * Roll back computations
  *****************************************************************/
    for(nswaps--; nswaps > mincutorder; nswaps--)
    {
        higain = swaps[nswaps];

        to = where[higain] = (where[higain] + 1) % 2;
        SWAP(id[higain], ed[higain], tmp);
        if(ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
            BNDDelete(nbnd, bndind, bndptr, higain);
        else if(ed[higain] > 0 && bndptr[higain] == -1)
            BNDInsert(nbnd, bndind, bndptr, higain);

        iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
        iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + ((to + 1) % 2) * ncon, 1);
        for(j = xadj[higain]; j < xadj[higain + 1]; j++)
        {
            k = adjncy[j];

            kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
            INC_DEC(id[k], ed[k], kwgt);

            if(bndptr[k] != -1 && ed[k] == 0)
                BNDDelete(nbnd, bndind, bndptr, k);
            if(bndptr[k] == -1 && ed[k] > 0)
                BNDInsert(nbnd, bndind, bndptr, k);
        }
    }

    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("\tMincut: %6" PRIDX " at %5" PRIDX ", NBND: %6" PRIDX ", NPwgts: [",
               mincut,
               mincutorder,
               nbnd);
        for(l = 0; l < ncon; l++)
            printf("(%6" PRIDX ", %6" PRIDX ") ", pwgts[l], pwgts[ncon + l]);
        printf("], LB: %.3" PRREAL "\n", ComputeLoadImbalance(graph, 2, ctrl->pijbm));
    }

    graph->mincut = mincut;
    graph->nbnd   = nbnd;


    for(i = 0; i < 2 * ncon; i++)
        rpqDestroy(queues[i]);

    WCOREPOP;
}

/************************ checkgraph.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * checkgraph.c
 *
 * This file contains routines related to I/O
 *
 * Started 8/28/94
 * George
 *
 */


/*************************************************************************/
/*! This function checks if a graph is valid. A valid graph must satisfy
    the following constraints:
    - It should contain no self-edges.
    - It should be undirected; i.e., (u,v) and (v,u) should be present.
    - The adjacency list should not contain multiple edges to the same
      other vertex.

    \param graph is the graph to be checked, whose numbering starts from 0.
    \param numflag is 0 if error reporting will be done using 0 as the
           numbering, or 1 if the reporting should be done using 1.
    \param verbose is 1 the identified errors will be displayed, or 0, if
           it should run silently.
*/
/*************************************************************************/
int CheckGraph(graph_t* graph, int numflag, int verbose)
{
    idx_t  i, j, k, l;
    idx_t  nvtxs, err = 0;
    idx_t  minedge, maxedge, minewgt, maxewgt;
    idx_t *xadj, *adjncy, *adjwgt, *htable;

    numflag = (numflag == 0 ? 0 : 1); /* make sure that numflag is 0 or 1 */

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    htable = ismalloc(nvtxs, 0, "htable");

    if(graph->nedges > 0)
    {
        minedge = maxedge = adjncy[0];
        if(adjwgt)
            minewgt = maxewgt = adjwgt[0];
    }

    for(i = 0; i < nvtxs; i++)
    {
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];

            minedge = (k < minedge) ? k : minedge;
            maxedge = (k > maxedge) ? k : maxedge;
            if(adjwgt)
            {
                minewgt = (adjwgt[j] < minewgt) ? adjwgt[j] : minewgt;
                maxewgt = (adjwgt[j] > maxewgt) ? adjwgt[j] : maxewgt;
            }

            if(i == k)
            {
                if(verbose)
                    printf("Vertex %" PRIDX
                           " contains a self-loop "
                           "(i.e., diagonal entry in the matrix)!\n",
                           i + numflag);
                err++;
            }
            else
            {
                for(l = xadj[k]; l < xadj[k + 1]; l++)
                {
                    if(adjncy[l] == i)
                    {
                        if(adjwgt)
                        {
                            if(adjwgt[l] != adjwgt[j])
                            {
                                if(verbose)
                                    printf("Edges (u:%" PRIDX " v:%" PRIDX " wgt:%" PRIDX
                                           ") and "
                                           "(v:%" PRIDX " u:%" PRIDX " wgt:%" PRIDX
                                           ") "
                                           "do not have the same weight!\n",
                                           i + numflag,
                                           k + numflag,
                                           adjwgt[j],
                                           k + numflag,
                                           i + numflag,
                                           adjwgt[l]);
                                err++;
                            }
                        }
                        break;
                    }
                }
                if(l == xadj[k + 1])
                {
                    if(verbose)
                        printf("Missing edge: (%" PRIDX " %" PRIDX ")!\n", k + numflag, i + numflag);
                    err++;
                }
            }

            if(htable[k] == 0)
            {
                htable[k]++;
            }
            else
            {
                if(verbose)
                    printf("Edge %" PRIDX " from vertex %" PRIDX
                           " is repeated %" PRIDX " times\n",
                           k + numflag,
                           i + numflag,
                           htable[k]++);
                err++;
            }
        }

        for(j = xadj[i]; j < xadj[i + 1]; j++)
            htable[adjncy[j]] = 0;
    }


    if(err > 0 && verbose)
    {
        printf("A total of %" PRIDX
               " errors exist in the input file. "
               "Correct them, and run again!\n",
               err);
    }

    gk_free((void**)&htable, LTERM);

    return (err == 0 ? 1 : 0);
}


/*************************************************************************/
/*! This function performs a quick check of the weights of the graph */
/*************************************************************************/
int CheckInputGraphWeights(
    idx_t nvtxs, idx_t ncon, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt)
{
    idx_t i;

    if(ncon <= 0)
    {
        printf("Input Error: ncon must be >= 1.\n");
        return 0;
    }

    if(vwgt)
    {
        for(i = ncon * nvtxs - 1; i >= 0; i--)
        {
            if(vwgt[i] < 0)
            {
                printf("Input Error: negative vertex weight(s).\n");
                return 0;
            }
        }
    }
    if(vsize)
    {
        for(i = nvtxs - 1; i >= 0; i--)
        {
            if(vsize[i] < 0)
            {
                printf("Input Error: negative vertex sizes(s).\n");
                return 0;
            }
        }
    }
    if(adjwgt)
    {
        for(i = xadj[nvtxs] - 1; i >= 0; i--)
        {
            if(adjwgt[i] < 0)
            {
                printf("Input Error: non-positive edge weight(s).\n");
                return 0;
            }
        }
    }

    return 1;
}


/*************************************************************************/
/*! This function creates a graph whose topology is consistent with
    Metis' requirements that:
    - There are no self-edges.
    - It is undirected; i.e., (u,v) and (v,u) should be present and of the
      same weight.
    - The adjacency list should not contain multiple edges to the same
      other vertex.

    Any of the above errors are fixed by performing the following operations:
    - Self-edges are removed.
    - The undirected graph is formed by the union of edges.
    - One of the duplicate edges is selected.

    The routine does not change the provided vertex weights.
*/
/*************************************************************************/
graph_t* FixGraph(graph_t* graph)
{
    idx_t    i, j, k, l, nvtxs, nedges;
    idx_t *  xadj, *adjncy, *adjwgt;
    idx_t *  nxadj, *nadjncy, *nadjwgt;
    graph_t* ngraph;
    uvw_t*   edges;


    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    ASSERT(adjwgt != NULL);

    ngraph = CreateGraph();

    ngraph->nvtxs = nvtxs;

    /* deal with vertex weights/sizes */
    ngraph->ncon = graph->ncon;
    ngraph->vwgt =
        icopy(nvtxs * graph->ncon, graph->vwgt, imalloc(nvtxs * graph->ncon, "FixGraph: vwgt"));

    ngraph->vsize = ismalloc(nvtxs, 1, "FixGraph: vsize");
    if(graph->vsize)
        icopy(nvtxs, graph->vsize, ngraph->vsize);

    /* fix graph by sorting the "superset" of edges */
    edges = (uvw_t*)gk_malloc(sizeof(uvw_t) * 2 * xadj[nvtxs], "FixGraph: edges");

    for(nedges = 0, i = 0; i < nvtxs; i++)
    {
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            /* keep only the upper-trianglular part of the adjacency matrix */
            if(i < adjncy[j])
            {
                edges[nedges].u = i;
                edges[nedges].v = adjncy[j];
                edges[nedges].w = adjwgt[j];
                nedges++;
            }
            else if(i > adjncy[j])
            {
                edges[nedges].u = adjncy[j];
                edges[nedges].v = i;
                edges[nedges].w = adjwgt[j];
                nedges++;
            }
        }
    }

    uvwsorti(nedges, edges);


    /* keep the unique subset */
    for(k = 0, i = 1; i < nedges; i++)
    {
        if(edges[k].v != edges[i].v || edges[k].u != edges[i].u)
        {
            edges[++k] = edges[i];
        }
    }
    nedges = k + 1;

    /* allocate memory for the fixed graph */
    nxadj = ngraph->xadj = ismalloc(nvtxs + 1, 0, "FixGraph: nxadj");
    nadjncy = ngraph->adjncy = imalloc(2 * nedges, "FixGraph: nadjncy");
    nadjwgt = ngraph->adjwgt = imalloc(2 * nedges, "FixGraph: nadjwgt");

    /* create the adjacency list of the fixed graph from the upper-triangular
     part of the adjacency matrix */
    for(k = 0; k < nedges; k++)
    {
        nxadj[edges[k].u]++;
        nxadj[edges[k].v]++;
    }
    MAKECSR(i, nvtxs, nxadj);

    for(k = 0; k < nedges; k++)
    {
        nadjncy[nxadj[edges[k].u]] = edges[k].v;
        nadjncy[nxadj[edges[k].v]] = edges[k].u;
        nadjwgt[nxadj[edges[k].u]] = edges[k].w;
        nadjwgt[nxadj[edges[k].v]] = edges[k].w;
        nxadj[edges[k].u]++;
        nxadj[edges[k].v]++;
    }
    SHIFTCSR(i, nvtxs, nxadj);

    gk_free((void**)&edges, LTERM);

    return ngraph;
}

/************************ debug.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * debug.c
 *
 * This file contains code that performs self debugging
 *
 * Started 7/24/97
 * George
 *
 */


/*************************************************************************/
/*! This function computes the total edgecut
 */
/*************************************************************************/
idx_t ComputeCut(graph_t* graph, idx_t* where)
{
    idx_t i, j, cut;

    if(graph->adjwgt == NULL)
    {
        for(cut = 0, i = 0; i < graph->nvtxs; i++)
        {
            for(j = graph->xadj[i]; j < graph->xadj[i + 1]; j++)
                if(where[i] != where[graph->adjncy[j]])
                    cut++;
        }
    }
    else
    {
        for(cut = 0, i = 0; i < graph->nvtxs; i++)
        {
            for(j = graph->xadj[i]; j < graph->xadj[i + 1]; j++)
                if(where[i] != where[graph->adjncy[j]])
                    cut += graph->adjwgt[j];
        }
    }

    return cut / 2;
}


/*************************************************************************/
/*! This function computes the total volume
 */
/*************************************************************************/
idx_t ComputeVolume(graph_t* graph, idx_t* where)
{
    idx_t  i, j, k, me, nvtxs, nparts, totalv;
    idx_t *xadj, *adjncy, *vsize, *marker;


    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vsize  = graph->vsize;

    nparts = where[iargmax(nvtxs, where, 1)] + 1;
    marker = ismalloc(nparts, -1, "ComputeVolume: marker");

    totalv = 0;

    for(i = 0; i < nvtxs; i++)
    {
        marker[where[i]] = i;
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = where[adjncy[j]];
            if(marker[k] != i)
            {
                marker[k] = i;
                totalv += (vsize ? vsize[i] : 1);
            }
        }
    }

    gk_free((void**)&marker, LTERM);

    return totalv;
}


/*************************************************************************/
/*! This function computes the cut given the graph and a where vector
 */
/*************************************************************************/
idx_t ComputeMaxCut(graph_t* graph, idx_t nparts, idx_t* where)
{
    idx_t  i, j, maxcut;
    idx_t* cuts;

    cuts = ismalloc(nparts, 0, "ComputeMaxCut: cuts");

    if(graph->adjwgt == NULL)
    {
        for(i = 0; i < graph->nvtxs; i++)
        {
            for(j = graph->xadj[i]; j < graph->xadj[i + 1]; j++)
                if(where[i] != where[graph->adjncy[j]])
                    cuts[where[i]]++;
        }
    }
    else
    {
        for(i = 0; i < graph->nvtxs; i++)
        {
            for(j = graph->xadj[i]; j < graph->xadj[i + 1]; j++)
                if(where[i] != where[graph->adjncy[j]])
                    cuts[where[i]] += graph->adjwgt[j];
        }
    }

    maxcut = cuts[iargmax(nparts, cuts, 1)];

    printf("%zu => %" PRIDX "\n", iargmax(nparts, cuts, 1), maxcut);

    gk_free((void**)&cuts, LTERM);

    return maxcut;
}


/*************************************************************************/
/*! This function checks whether or not the boundary information is correct
 */
/*************************************************************************/
idx_t CheckBnd(graph_t* graph)
{
    idx_t  i, j, nvtxs, nbnd;
    idx_t *xadj, *adjncy, *where, *bndptr, *bndind;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    where  = graph->where;
    bndptr = graph->bndptr;
    bndind = graph->bndind;

    for(nbnd = 0, i = 0; i < nvtxs; i++)
    {
        if(xadj[i + 1] - xadj[i] == 0)
            nbnd++; /* Islands are considered to be boundary vertices */

        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            if(where[i] != where[adjncy[j]])
            {
                nbnd++;
                ASSERT(bndptr[i] != -1);
                ASSERT(bndind[bndptr[i]] == i);
                break;
            }
        }
    }

    ASSERTP(nbnd == graph->nbnd, ("%" PRIDX " %" PRIDX "\n", nbnd, graph->nbnd));

    return 1;
}


/*************************************************************************/
/*! This function checks whether or not the boundary information is correct
 */
/*************************************************************************/
idx_t CheckBnd2(graph_t* graph)
{
    idx_t  i, j, nvtxs, nbnd, id, ed;
    idx_t *xadj, *adjncy, *where, *bndptr, *bndind;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    where  = graph->where;
    bndptr = graph->bndptr;
    bndind = graph->bndind;

    for(nbnd = 0, i = 0; i < nvtxs; i++)
    {
        id = ed = 0;
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            if(where[i] != where[adjncy[j]])
                ed += graph->adjwgt[j];
            else
                id += graph->adjwgt[j];
        }
        if(ed - id >= 0 && xadj[i] < xadj[i + 1])
        {
            nbnd++;
            ASSERTP(bndptr[i] != -1, ("%" PRIDX " %" PRIDX " %" PRIDX "\n", i, id, ed));
            ASSERT(bndind[bndptr[i]] == i);
        }
    }

    ASSERTP(nbnd == graph->nbnd, ("%" PRIDX " %" PRIDX "\n", nbnd, graph->nbnd));

    return 1;
}


/*************************************************************************/
/*! This function checks whether or not the boundary information is correct
 */
/*************************************************************************/
idx_t CheckNodeBnd(graph_t* graph, idx_t onbnd)
{
    idx_t  i, j, nvtxs, nbnd;
    idx_t *xadj, *adjncy, *where, *bndptr, *bndind;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    where  = graph->where;
    bndptr = graph->bndptr;
    bndind = graph->bndind;

    for(nbnd = 0, i = 0; i < nvtxs; i++)
    {
        if(where[i] == 2)
            nbnd++;
    }

    ASSERTP(nbnd == onbnd, ("%" PRIDX " %" PRIDX "\n", nbnd, onbnd));

    for(i = 0; i < nvtxs; i++)
    {
        if(where[i] != 2)
        {
            ASSERTP(bndptr[i] == -1, ("%" PRIDX " %" PRIDX "\n", i, bndptr[i]));
        }
        else
        {
            ASSERTP(bndptr[i] != -1, ("%" PRIDX " %" PRIDX "\n", i, bndptr[i]));
        }
    }

    return 1;
}


/*************************************************************************/
/*! This function checks whether or not the rinfo of a vertex is consistent
 */
/*************************************************************************/
idx_t CheckRInfo(ctrl_t* ctrl, ckrinfo_t* rinfo)
{
    idx_t   i, j;
    cnbr_t* nbrs;

    ASSERT(ctrl->nbrpoolcpos >= 0);
    ASSERT(rinfo->nnbrs < ctrl->nparts);

    nbrs = ctrl->cnbrpool + rinfo->inbr;

    for(i = 0; i < rinfo->nnbrs; i++)
    {
        for(j = i + 1; j < rinfo->nnbrs; j++)
            ASSERTP(nbrs[i].pid != nbrs[j].pid,
                    ("%" PRIDX " %" PRIDX " %" PRIDX " %" PRIDX "\n",
                     i,
                     j,
                     nbrs[i].pid,
                     nbrs[j].pid));
    }

    return 1;
}


/*************************************************************************/
/*! This function checks the correctness of the NodeFM data structures
 */
/*************************************************************************/
idx_t CheckNodePartitionParams(graph_t* graph)
{
    idx_t  i, j, k, l, nvtxs, me, other;
    idx_t *xadj, *adjncy, *adjwgt, *vwgt, *where;
    idx_t  edegrees[2], pwgts[3];

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    where  = graph->where;

    /*------------------------------------------------------------
  / Compute now the separator external degrees
  /------------------------------------------------------------*/
    pwgts[0] = pwgts[1] = pwgts[2] = 0;
    for(i = 0; i < nvtxs; i++)
    {
        me = where[i];
        pwgts[me] += vwgt[i];

        if(me == 2)
        { /* If it is on the separator do some computations */
            edegrees[0] = edegrees[1] = 0;

            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                other = where[adjncy[j]];
                if(other != 2)
                    edegrees[other] += vwgt[adjncy[j]];
            }
            if(edegrees[0] != graph->nrinfo[i].edegrees[0]
               || edegrees[1] != graph->nrinfo[i].edegrees[1])
            {
                printf("Something wrong with edegrees: %" PRIDX " %" PRIDX
                       " %" PRIDX " %" PRIDX " %" PRIDX "\n",
                       i,
                       edegrees[0],
                       edegrees[1],
                       graph->nrinfo[i].edegrees[0],
                       graph->nrinfo[i].edegrees[1]);
                return 0;
            }
        }
    }

    if(pwgts[0] != graph->pwgts[0] || pwgts[1] != graph->pwgts[1]
       || pwgts[2] != graph->pwgts[2])
    {
        printf("Something wrong with part-weights: %" PRIDX " %" PRIDX
               " %" PRIDX " %" PRIDX " %" PRIDX " %" PRIDX "\n",
               pwgts[0],
               pwgts[1],
               pwgts[2],
               graph->pwgts[0],
               graph->pwgts[1],
               graph->pwgts[2]);
        return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function checks if the separator is indeed a separator
 */
/*************************************************************************/
idx_t IsSeparable(graph_t* graph)
{
    idx_t  i, j, nvtxs, other;
    idx_t *xadj, *adjncy, *where;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    where  = graph->where;

    for(i = 0; i < nvtxs; i++)
    {
        if(where[i] == 2)
            continue;
        other = (where[i] + 1) % 2;
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            ASSERTP(where[adjncy[j]] != other,
                    ("%" PRIDX " %" PRIDX " %" PRIDX " %" PRIDX " %" PRIDX " %" PRIDX "\n",
                     i,
                     where[i],
                     adjncy[j],
                     where[adjncy[j]],
                     xadj[i + 1] - xadj[i],
                     xadj[adjncy[j] + 1] - xadj[adjncy[j]]));
        }
    }

    return 1;
}


/*************************************************************************/
/*! This function recomputes the vrinfo fields and checks them against
    those in the graph->vrinfo structure */
/*************************************************************************/
void CheckKWayVolPartitionParams(ctrl_t* ctrl, graph_t* graph)
{
    idx_t      i, ii, j, k, kk, l, nvtxs, nbnd, mincut, minvol, me, other, pid;
    idx_t *    xadj, *vsize, *adjncy, *pwgts, *where, *bndind, *bndptr;
    vkrinfo_t *rinfo, *myrinfo, *orinfo, tmprinfo;
    vnbr_t *   mynbrs, *onbrs, *tmpnbrs;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vsize  = graph->vsize;
    adjncy = graph->adjncy;
    where  = graph->where;
    rinfo  = graph->vkrinfo;

    tmpnbrs = (vnbr_t*)wspacemalloc(ctrl, ctrl->nparts * sizeof(vnbr_t));

    /*------------------------------------------------------------
  / Compute now the iv/ev degrees
  /------------------------------------------------------------*/
    for(i = 0; i < nvtxs; i++)
    {
        me = where[i];

        myrinfo = rinfo + i;
        mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

        for(k = 0; k < myrinfo->nnbrs; k++)
            tmpnbrs[k] = mynbrs[k];

        tmprinfo.nnbrs = myrinfo->nnbrs;
        tmprinfo.nid   = myrinfo->nid;
        tmprinfo.ned   = myrinfo->ned;

        myrinfo = &tmprinfo;
        mynbrs  = tmpnbrs;

        for(k = 0; k < myrinfo->nnbrs; k++)
            mynbrs[k].gv = 0;

        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            ii     = adjncy[j];
            other  = where[ii];
            orinfo = rinfo + ii;
            onbrs  = ctrl->vnbrpool + orinfo->inbr;

            if(me == other)
            {
                /* Find which domains 'i' is connected and 'ii' is not and update their gain */
                for(k = 0; k < myrinfo->nnbrs; k++)
                {
                    pid = mynbrs[k].pid;
                    for(kk = 0; kk < orinfo->nnbrs; kk++)
                    {
                        if(onbrs[kk].pid == pid)
                            break;
                    }
                    if(kk == orinfo->nnbrs)
                        mynbrs[k].gv -= vsize[ii];
                }
            }
            else
            {
                /* Find the orinfo[me].ed and see if I'm the only connection */
                for(k = 0; k < orinfo->nnbrs; k++)
                {
                    if(onbrs[k].pid == me)
                        break;
                }

                if(onbrs[k].ned == 1)
                { /* I'm the only connection of 'ii' in 'me' */
                    for(k = 0; k < myrinfo->nnbrs; k++)
                    {
                        if(mynbrs[k].pid == other)
                        {
                            mynbrs[k].gv += vsize[ii];
                            break;
                        }
                    }

                    /* Increase the gains for all the common domains between 'i' and 'ii' */
                    for(k = 0; k < myrinfo->nnbrs; k++)
                    {
                        if((pid = mynbrs[k].pid) == other)
                            continue;
                        for(kk = 0; kk < orinfo->nnbrs; kk++)
                        {
                            if(onbrs[kk].pid == pid)
                            {
                                mynbrs[k].gv += vsize[ii];
                                break;
                            }
                        }
                    }
                }
                else
                {
                    /* Find which domains 'i' is connected and 'ii' is not and update their gain */
                    for(k = 0; k < myrinfo->nnbrs; k++)
                    {
                        if((pid = mynbrs[k].pid) == other)
                            continue;
                        for(kk = 0; kk < orinfo->nnbrs; kk++)
                        {
                            if(onbrs[kk].pid == pid)
                                break;
                        }
                        if(kk == orinfo->nnbrs)
                            mynbrs[k].gv -= vsize[ii];
                    }
                }
            }
        }

        myrinfo = rinfo + i;
        mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

        for(k = 0; k < myrinfo->nnbrs; k++)
        {
            pid = mynbrs[k].pid;
            for(kk = 0; kk < tmprinfo.nnbrs; kk++)
            {
                if(tmpnbrs[kk].pid == pid)
                {
                    if(tmpnbrs[kk].gv != mynbrs[k].gv)
                        printf("[%8" PRIDX " %8" PRIDX " %8" PRIDX " %+8" PRIDX
                               " %+8" PRIDX "]\n",
                               i,
                               where[i],
                               pid,
                               mynbrs[k].gv,
                               tmpnbrs[kk].gv);
                    break;
                }
            }
        }
    }

    WCOREPOP;
}


/************************ mcutil.c ************************/
/*
 * mutil.c
 *
 * This file contains various utility functions for the MOC portion of the
 * code
 *
 * Started 2/15/98
 * George
 *
 * $Id: mcutil.c 13901 2013-03-24 16:17:03Z karypis $
 *
 */


/*************************************************************************/
/*! This function compares two vectors x & y and returns true
    if \forall i, x[i] <= y[i].
*/
/**************************************************************************/
int rvecle(idx_t n, real_t* x, real_t* y)
{
    for(n--; n >= 0; n--)
    {
        if(x[n] > y[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function compares two vectors x & y and returns true
    if \forall i, x[i] >= y[i].
*/
/**************************************************************************/
int rvecge(idx_t n, real_t* x, real_t* y)
{
    for(n--; n >= 0; n--)
    {
        if(x[n] < y[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function compares vectors x1+x2 against y and returns true
    if \forall i, x1[i]+x2[i] <= y[i].
*/
/**************************************************************************/
int rvecsumle(idx_t n, real_t* x1, real_t* x2, real_t* y)
{
    for(n--; n >= 0; n--)
    {
        if(x1[n] + x2[n] > y[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function returns max_i(x[i]-y[i]) */
/**************************************************************************/
real_t rvecmaxdiff(idx_t n, real_t* x, real_t* y)
{
    real_t max;

    max = x[0] - y[0];

    for(n--; n > 0; n--)
    {
        if(max < x[n] - y[n])
            max = x[n] - y[n];
    }

    return max;
}


/*************************************************************************/
/*! This function returns true if \forall i, x[i] <= z[i]. */
/**************************************************************************/
int ivecle(idx_t n, idx_t* x, idx_t* z)
{
    for(n--; n >= 0; n--)
    {
        if(x[n] > z[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function returns true if \forall i, x[i] >= z[i]. */
/**************************************************************************/
int ivecge(idx_t n, idx_t* x, idx_t* z)
{
    for(n--; n >= 0; n--)
    {
        if(x[n] < z[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function returns true if \forall i, a*x[i]+y[i] <= z[i]. */
/**************************************************************************/
int ivecaxpylez(idx_t n, idx_t a, idx_t* x, idx_t* y, idx_t* z)
{
    for(n--; n >= 0; n--)
    {
        if(a * x[n] + y[n] > z[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function returns true if \forall i, a*x[i]+y[i] >= z[i]. */
/**************************************************************************/
int ivecaxpygez(idx_t n, idx_t a, idx_t* x, idx_t* y, idx_t* z)
{
    for(n--; n >= 0; n--)
    {
        if(a * x[n] + y[n] < z[n])
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function checks if v+u2 provides a better balance in the weight
     vector that v+u1 */
/*************************************************************************/
int BetterVBalance(idx_t ncon, real_t* invtvwgt, idx_t* v_vwgt, idx_t* u1_vwgt, idx_t* u2_vwgt)
{
    idx_t  i;
    real_t sum1 = 0.0, sum2 = 0.0, diff1 = 0.0, diff2 = 0.0;

    for(i = 0; i < ncon; i++)
    {
        sum1 += (v_vwgt[i] + u1_vwgt[i]) * invtvwgt[i];
        sum2 += (v_vwgt[i] + u2_vwgt[i]) * invtvwgt[i];
    }
    sum1 = sum1 / ncon;
    sum2 = sum2 / ncon;

    for(i = 0; i < ncon; i++)
    {
        diff1 += rabs(sum1 - (v_vwgt[i] + u1_vwgt[i]) * invtvwgt[i]);
        diff2 += rabs(sum2 - (v_vwgt[i] + u2_vwgt[i]) * invtvwgt[i]);
    }

    return (diff1 - diff2 >= 0);
}


/*************************************************************************/
/*! This function takes two ubfactor-centered load imbalance vectors x & y,
    and returns true if y is better balanced than x. */
/*************************************************************************/
int BetterBalance2Way(idx_t n, real_t* x, real_t* y)
{
    real_t nrm1 = 0.0, nrm2 = 0.0;

    for(--n; n >= 0; n--)
    {
        if(x[n] > 0)
            nrm1 += x[n] * x[n];
        if(y[n] > 0)
            nrm2 += y[n] * y[n];
    }
    return nrm2 < nrm1;
}


/*************************************************************************/
/*! Given a vertex and two weights, this function returns 1, if the second
    partition will be more balanced than the first after the weighted
    additional of that vertex.
    The balance determination takes into account the ideal target weights
    of the two partitions.
*/
/*************************************************************************/
int BetterBalanceKWay(idx_t   ncon,
                      idx_t*  vwgt,
                      real_t* ubvec,
                      idx_t   a1,
                      idx_t*  pt1,
                      real_t* bm1,
                      idx_t   a2,
                      idx_t*  pt2,
                      real_t* bm2)
{
    idx_t  i;
    real_t tmp, nrm1 = 0.0, nrm2 = 0.0, max1 = 0.0, max2 = 0.0;

    for(i = 0; i < ncon; i++)
    {
        tmp = bm1[i] * (pt1[i] + a1 * vwgt[i]) - ubvec[i];
        //printf("BB: %d %+.4f ", (int)i, (float)tmp);
        nrm1 += tmp * tmp;
        max1 = (tmp > max1 ? tmp : max1);

        tmp = bm2[i] * (pt2[i] + a2 * vwgt[i]) - ubvec[i];
        //printf("%+.4f ", (float)tmp);
        nrm2 += tmp * tmp;
        max2 = (tmp > max2 ? tmp : max2);

        //printf("%4d %4d %4d %4d %4d %4d %4d %.2f\n",
        //    (int)vwgt[i],
        //    (int)a1, (int)pt1[i], (int)tpt1[i],
        //    (int)a2, (int)pt2[i], (int)tpt2[i], ubvec[i]);
    }
    //printf("   %.3f %.3f %.3f %.3f\n", (float)max1, (float)nrm1, (float)max2, (float)nrm2);

    if(max2 < max1)
        return 1;

    if(max2 == max1 && nrm2 < nrm1)
        return 1;

    return 0;
}


/*************************************************************************/
/*! Computes the maximum load imbalance of a partitioning solution over
    all the constraints. */
/**************************************************************************/
real_t ComputeLoadImbalance(graph_t* graph, idx_t nparts, real_t* pijbm)
{
    idx_t  i, j, ncon, *pwgts;
    real_t max, cur;

    ncon  = graph->ncon;
    pwgts = graph->pwgts;

    max = 1.0;
    for(i = 0; i < ncon; i++)
    {
        for(j = 0; j < nparts; j++)
        {
            cur = pwgts[j * ncon + i] * pijbm[j * ncon + i];
            if(cur > max)
                max = cur;
        }
    }

    return max;
}


/*************************************************************************/
/*! Computes the maximum load imbalance difference of a partitioning
    solution over all the constraints.
    The difference is defined with respect to the allowed maximum
    unbalance for the respective constraint.
 */
/**************************************************************************/
real_t ComputeLoadImbalanceDiff(graph_t* graph, idx_t nparts, real_t* pijbm, real_t* ubvec)
{
    idx_t  i, j, ncon, *pwgts;
    real_t max, cur;

    ncon  = graph->ncon;
    pwgts = graph->pwgts;

    max = -1.0;
    for(i = 0; i < ncon; i++)
    {
        for(j = 0; j < nparts; j++)
        {
            cur = pwgts[j * ncon + i] * pijbm[j * ncon + i] - ubvec[i];
            if(cur > max)
                max = cur;
        }
    }

    return max;
}


/*************************************************************************/
/*! Computes the difference between load imbalance of each constraint across
    the partitions minus the desired upper bound on the load imabalnce.
    It also returns the maximum load imbalance across the partitions &
    constraints. */
/**************************************************************************/
real_t ComputeLoadImbalanceDiffVec(
    graph_t* graph, idx_t nparts, real_t* pijbm, real_t* ubfactors, real_t* diffvec)
{
    idx_t  i, j, ncon, *pwgts;
    real_t cur, max;

    ncon  = graph->ncon;
    pwgts = graph->pwgts;

    for(max = -1.0, i = 0; i < ncon; i++)
    {
        diffvec[i] = pwgts[i] * pijbm[i] - ubfactors[i];
        for(j = 1; j < nparts; j++)
        {
            cur = pwgts[j * ncon + i] * pijbm[j * ncon + i] - ubfactors[i];
            if(cur > diffvec[i])
                diffvec[i] = cur;
        }
        if(max < diffvec[i])
            max = diffvec[i];
    }

    return max;
}


/*************************************************************************/
/*! Computes the load imbalance of each constraint across the partitions. */
/**************************************************************************/
void ComputeLoadImbalanceVec(graph_t* graph, idx_t nparts, real_t* pijbm, real_t* lbvec)
{
    idx_t  i, j, ncon, *pwgts;
    real_t cur;

    ncon  = graph->ncon;
    pwgts = graph->pwgts;

    for(i = 0; i < ncon; i++)
    {
        lbvec[i] = pwgts[i] * pijbm[i];
        for(j = 1; j < nparts; j++)
        {
            cur = pwgts[j * ncon + i] * pijbm[j * ncon + i];
            if(cur > lbvec[i])
                lbvec[i] = cur;
        }
    }
}


/************************ minconn.c ************************/
/*!
\file
\brief Functions that deal with prunning the number of adjacent subdomains in kmetis

\date Started 7/15/98
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version $Id: minconn.c 17513 2014-08-05 16:20:50Z dominique $
*/


/*************************************************************************/
/*! This function computes the subdomain graph storing the result in the
    pre-allocated workspace arrays */
/*************************************************************************/
void ComputeSubDomainGraph(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, ii, j, pid, other, nparts, nvtxs, nnbrs;
    idx_t *xadj, *adjncy, *adjwgt, *where;
    idx_t *pptr, *pind;
    idx_t  nads = 0, *vadids, *vadwgts;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    where  = graph->where;

    nparts = ctrl->nparts;

    vadids  = ctrl->pvec1;
    vadwgts = iset(nparts, 0, ctrl->pvec2);

    pptr = iwspacemalloc(ctrl, nparts + 1);
    pind = iwspacemalloc(ctrl, nvtxs);
    iarray2csr(nvtxs, nparts, where, pptr, pind);

    for(pid = 0; pid < nparts; pid++)
    {
        switch(ctrl->objtype)
        {
            case METIS_OBJTYPE_CUT: {
                ckrinfo_t* rinfo;
                cnbr_t*    nbrs;

                rinfo = graph->ckrinfo;
                for(nads = 0, ii = pptr[pid]; ii < pptr[pid + 1]; ii++)
                {
                    i = pind[ii];
                    ASSERT(pid == where[i]);

                    if(rinfo[i].ed > 0)
                    {
                        nnbrs = rinfo[i].nnbrs;
                        nbrs  = ctrl->cnbrpool + rinfo[i].inbr;

                        for(j = 0; j < nnbrs; j++)
                        {
                            other = nbrs[j].pid;
                            if(vadwgts[other] == 0)
                                vadids[nads++] = other;
                            vadwgts[other] += nbrs[j].ed;
                        }
                    }
                }
            }
            break;

            case METIS_OBJTYPE_VOL: {
                vkrinfo_t* rinfo;
                vnbr_t*    nbrs;

                rinfo = graph->vkrinfo;
                for(nads = 0, ii = pptr[pid]; ii < pptr[pid + 1]; ii++)
                {
                    i = pind[ii];
                    ASSERT(pid == where[i]);

                    if(rinfo[i].ned > 0)
                    {
                        nnbrs = rinfo[i].nnbrs;
                        nbrs  = ctrl->vnbrpool + rinfo[i].inbr;

                        for(j = 0; j < nnbrs; j++)
                        {
                            other = nbrs[j].pid;
                            if(vadwgts[other] == 0)
                                vadids[nads++] = other;
                            vadwgts[other] += nbrs[j].ned;
                        }
                    }
                }
            }
            break;

            default:
                gk_errexit(SIGERR, "Unknown objtype: %d\n", ctrl->objtype);
        }

        /* See if you have enough memory to store the adjacent info for that subdomain */
        if(ctrl->maxnads[pid] < nads)
        {
            ctrl->maxnads[pid] = 2 * nads;
            ctrl->adids[pid] =
                irealloc(ctrl->adids[pid], ctrl->maxnads[pid], "ComputeSubDomainGraph: adids[pid]");
            ctrl->adwgts[pid] =
                irealloc(ctrl->adwgts[pid], ctrl->maxnads[pid], "ComputeSubDomainGraph: adids[pid]");
        }

        ctrl->nads[pid] = nads;
        for(j = 0; j < nads; j++)
        {
            ctrl->adids[pid][j]  = vadids[j];
            ctrl->adwgts[pid][j] = vadwgts[vadids[j]];

            vadwgts[vadids[j]] = 0;
        }
    }

    WCOREPOP;
}


/*************************************************************************/
/*! This function updates the weight of an edge in the subdomain graph by
    adding to it the value of ewgt. The update can either increase or
    decrease the weight of the subdomain edge based on the value of ewgt.

    \param u is the ID of one of the incident subdomains to the edge
    \param v is the ID of the other incident subdomains to the edge
    \param ewgt is the weight to be added to the subdomain edge
    \param nparts is the number of subdomains
    \param r_maxndoms is the maximum number of adjacent subdomains and is
           updated as necessary. The update is skipped if a NULL value is
           supplied.
*/
/*************************************************************************/
void UpdateEdgeSubDomainGraph(ctrl_t* ctrl, idx_t u, idx_t v, idx_t ewgt, idx_t* r_maxndoms)
{
    idx_t i, j, nads;

    if(ewgt == 0)
        return;

    for(i = 0; i < 2; i++)
    {
        nads = ctrl->nads[u];
        /* Find the edge */
        for(j = 0; j < nads; j++)
        {
            if(ctrl->adids[u][j] == v)
            {
                ctrl->adwgts[u][j] += ewgt;
                break;
            }
        }

        if(j == nads)
        {
            /* Deal with the case in which the edge was not found */
            ASSERT(ewgt > 0);
            if(ctrl->maxnads[u] == nads)
            {
                ctrl->maxnads[u] = 2 * (nads + 1);
                ctrl->adids[u]   = irealloc(ctrl->adids[u],
                                          ctrl->maxnads[u],
                                          "IncreaseEdgeSubDomainGraph: adids[pid]");
                ctrl->adwgts[u]  = irealloc(ctrl->adwgts[u],
                                           ctrl->maxnads[u],
                                           "IncreaseEdgeSubDomainGraph: adids[pid]");
            }
            ctrl->adids[u][nads]  = v;
            ctrl->adwgts[u][nads] = ewgt;
            nads++;
            if(r_maxndoms != NULL && nads > *r_maxndoms)
            {
                printf("You just increased the maxndoms: %" PRIDX " %" PRIDX "\n", nads, *r_maxndoms);
                *r_maxndoms = nads;
            }
        }
        else
        {
            /* See if the updated edge becomes 0 */
            ASSERT(ctrl->adwgts[u][j] >= 0);
            if(ctrl->adwgts[u][j] == 0)
            {
                ctrl->adids[u][j]  = ctrl->adids[u][nads - 1];
                ctrl->adwgts[u][j] = ctrl->adwgts[u][nads - 1];
                nads--;
                if(r_maxndoms != NULL && nads + 1 == *r_maxndoms)
                    *r_maxndoms = ctrl->nads[iargmax(ctrl->nparts, ctrl->nads, 1)];
            }
        }
        ctrl->nads[u] = nads;

        SWAP(u, v, j);
    }
}


/*************************************************************************/
/*! This function computes the subdomain graph */
/*************************************************************************/
void EliminateSubDomainEdges(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, ii, j, k, ncon, nparts, scheme, pid_from, pid_to, me, other, nvtxs,
        total, max, avg, totalout, nind = 0, ncand = 0, ncand2, target, target2,
                                   nadd, bestnadd = 0;
    idx_t  min, move, *cpwgt;
    idx_t *xadj, *adjncy, *vwgt, *adjwgt, *pwgts, *where, *maxpwgt, *mypmat,
        *otherpmat, *kpmat, *ind;
    idx_t * nads, **adids, **adwgts;
    ikv_t * cand, *cand2;
    ipq_t   queue;
    real_t *tpwgts, badfactor = 1.4;
    idx_t * pptr, *pind;
    idx_t *vmarker = NULL, *pmarker = NULL, *modind = NULL; /* volume specific work arrays */

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vwgt   = graph->vwgt;
    adjwgt = (ctrl->objtype == METIS_OBJTYPE_VOL ? NULL : graph->adjwgt);

    where = graph->where;
    pwgts = graph->pwgts; /* We assume that this is properly initialized */

    nparts = ctrl->nparts;
    tpwgts = ctrl->tpwgts;

    cpwgt     = iwspacemalloc(ctrl, ncon);
    maxpwgt   = iwspacemalloc(ctrl, nparts * ncon);
    ind       = iwspacemalloc(ctrl, nvtxs);
    otherpmat = iset(nparts, 0, iwspacemalloc(ctrl, nparts));

    cand  = ikvwspacemalloc(ctrl, nparts);
    cand2 = ikvwspacemalloc(ctrl, nparts);

    pptr = iwspacemalloc(ctrl, nparts + 1);
    pind = iwspacemalloc(ctrl, nvtxs);
    iarray2csr(nvtxs, nparts, where, pptr, pind);

    if(ctrl->objtype == METIS_OBJTYPE_VOL)
    {
        /* Vol-refinement specific working arrays */
        modind  = iwspacemalloc(ctrl, nvtxs);
        vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
        pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));
    }


    /* Compute the pmat matrix and ndoms */
    ComputeSubDomainGraph(ctrl, graph);

    nads   = ctrl->nads;
    adids  = ctrl->adids;
    adwgts = ctrl->adwgts;

    mypmat = iset(nparts, 0, ctrl->pvec1);
    kpmat  = iset(nparts, 0, ctrl->pvec2);

    /* Compute the maximum allowed weight for each domain */
    for(i = 0; i < nparts; i++)
    {
        for(j = 0; j < ncon; j++)
            maxpwgt[i * ncon + j] = (ncon == 1 ? 1.25 : 1.025) * tpwgts[i]
                                    * graph->tvwgt[j] * ctrl->ubfactors[j];
    }

    ipqInit(&queue, nparts);

    /* Get into the loop eliminating subdomain connections */
    while(1)
    {
        total = isum(nparts, nads, 1);
        avg   = total / nparts;
        max   = nads[iargmax(nparts, nads, 1)];

        IFSET(ctrl->dbglvl,
              METIS_DBG_CONNINFO,
              printf("Adjacent Subdomain Stats: Total: %3" PRIDX ", "
                     "Max: %3" PRIDX "[%zu], Avg: %3" PRIDX "\n",
                     total,
                     max,
                     iargmax(nparts, nads, 1),
                     avg));

        if(max < badfactor * avg)
            break;

        /* Add the subdomains that you will try to reduce their connectivity */
        ipqReset(&queue);
        for(i = 0; i < nparts; i++)
        {
            if(nads[i] >= avg + (max - avg) / 2)
                ipqInsert(&queue, i, nads[i]);
        }

        move = 0;
        while((me = ipqGetTop(&queue)) != -1)
        {
            totalout = isum(nads[me], adwgts[me], 1);

            for(ncand2 = 0, i = 0; i < nads[me]; i++)
            {
                mypmat[adids[me][i]] = adwgts[me][i];

                /* keep track of the weakly connected adjacent subdomains */
                if(2 * nads[me] * adwgts[me][i] < totalout)
                {
                    cand2[ncand2].val   = adids[me][i];
                    cand2[ncand2++].key = adwgts[me][i];
                }
            }

            IFSET(ctrl->dbglvl,
                  METIS_DBG_CONNINFO,
                  printf("Me: %" PRIDX ", Degree: %4" PRIDX ", TotalOut: %" PRIDX ",\n",
                         me,
                         nads[me],
                         totalout));

            /* Sort the connections according to their cut */
            ikvsorti(ncand2, cand2);

            /* Two schemes are used for eliminating subdomain edges.
         The first, tries to eliminate subdomain edges by moving remote groups
         of vertices to subdomains that 'me' is already connected to.
         The second, tries to eliminate subdomain edges by moving entire sets of
         my vertices that connect to the 'other' subdomain to a subdomain that
         I'm already connected to.
         These two schemes are applied in sequence. */
            target = target2 = -1;
            for(scheme = 0; scheme < 2; scheme++)
            {
                for(min = 0; min < ncand2; min++)
                {
                    other = cand2[min].val;

                    /* pid_from is the subdomain from where the vertices will be removed.
             pid_to is the adjacent subdomain to pid_from that defines the
             (me, other) subdomain edge that needs to be removed */
                    if(scheme == 0)
                    {
                        pid_from = other;
                        pid_to   = me;
                    }
                    else
                    {
                        pid_from = me;
                        pid_to   = other;
                    }

                    /* Go and find the vertices in 'other' that are connected in 'me' */
                    for(nind = 0, ii = pptr[pid_from]; ii < pptr[pid_from + 1]; ii++)
                    {
                        i = pind[ii];
                        ASSERT(where[i] == pid_from);
                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            if(where[adjncy[j]] == pid_to)
                            {
                                ind[nind++] = i;
                                break;
                            }
                        }
                    }

                    /* Go and construct the otherpmat to see where these nind vertices are
             connected to */
                    iset(ncon, 0, cpwgt);
                    for(ncand = 0, ii = 0; ii < nind; ii++)
                    {
                        i = ind[ii];
                        iaxpy(ncon, 1, vwgt + i * ncon, 1, cpwgt, 1);

                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            if((k = where[adjncy[j]]) == pid_from)
                                continue;
                            if(otherpmat[k] == 0)
                                cand[ncand++].val = k;
                            otherpmat[k] += (adjwgt ? adjwgt[j] : 1);
                        }
                    }

                    for(i = 0; i < ncand; i++)
                    {
                        cand[i].key = otherpmat[cand[i].val];
                        ASSERT(cand[i].key > 0);
                    }

                    ikvsortd(ncand, cand);

                    IFSET(ctrl->dbglvl,
                          METIS_DBG_CONNINFO,
                          printf("\tMinOut: %4" PRIDX ", to: %3" PRIDX
                                 ", TtlWgt: %5" PRIDX "[#:%" PRIDX "]\n",
                                 mypmat[other],
                                 other,
                                 isum(ncon, cpwgt, 1),
                                 nind));

                    /* Go through and select the first domain that is common with 'me', and does
             not increase the nads[target] higher than nads[me], subject to the maxpwgt
             constraint. Traversal is done from the mostly connected to the least. */
                    for(i = 0; i < ncand; i++)
                    {
                        k = cand[i].val;

                        if(mypmat[k] > 0)
                        {
                            /* Check if balance will go off */
                            if(!ivecaxpylez(ncon, 1, cpwgt, pwgts + k * ncon, maxpwgt + k * ncon))
                                continue;

                            /* get a dense vector out of k's connectivity */
                            for(j = 0; j < nads[k]; j++)
                                kpmat[adids[k][j]] = adwgts[k][j];

                            /* Check if the move to domain k will increase the nads of another
                 subdomain j that the set of vertices being moved are connected
                 to but domain k is not connected to. */
                            for(j = 0; j < nparts; j++)
                            {
                                if(otherpmat[j] > 0 && kpmat[j] == 0
                                   && nads[j] + 1 >= nads[me])
                                    break;
                            }

                            /* There were no bad second level effects. See if you can find a
                 subdomain to move to. */
                            if(j == nparts)
                            {
                                for(nadd = 0, j = 0; j < nparts; j++)
                                {
                                    if(otherpmat[j] > 0 && kpmat[j] == 0)
                                        nadd++;
                                }

                                IFSET(ctrl->dbglvl,
                                      METIS_DBG_CONNINFO,
                                      printf("\t\tto=%" PRIDX ", nadd=%" PRIDX ", %" PRIDX "\n",
                                             k,
                                             nadd,
                                             nads[k]));

                                if(nads[k] + nadd < nads[me])
                                {
                                    if(target2 == -1
                                       || nads[target2] + bestnadd > nads[k] + nadd
                                       || (nads[target2] + bestnadd == nads[k] + nadd
                                           && bestnadd > nadd))
                                    {
                                        target2  = k;
                                        bestnadd = nadd;
                                    }
                                }

                                if(nadd == 0)
                                    target = k;
                            }

                            /* reset kpmat for the next iteration */
                            for(j = 0; j < nads[k]; j++)
                                kpmat[adids[k][j]] = 0;
                        }

                        if(target != -1)
                            break;
                    }

                    /* reset the otherpmat for the next iteration */
                    for(i = 0; i < ncand; i++)
                        otherpmat[cand[i].val] = 0;

                    if(target == -1 && target2 != -1)
                        target = target2;

                    if(target != -1)
                    {
                        IFSET(ctrl->dbglvl,
                              METIS_DBG_CONNINFO,
                              printf("\t\tScheme: %" PRIDX ". Moving to %" PRIDX "\n", scheme, target));
                        move = 1;
                        break;
                    }
                }

                if(target != -1)
                    break; /* A move was found. No need to try the other scheme */
            }

            /* reset the mypmat for next iteration */
            for(i = 0; i < nads[me]; i++)
                mypmat[adids[me][i]] = 0;

            /* Note that once a target is found the above loops exit right away. So the
         following variables are valid */
            if(target != -1)
            {
                switch(ctrl->objtype)
                {
                    case METIS_OBJTYPE_CUT:
                        MoveGroupMinConnForCut(ctrl, graph, target, nind, ind);
                        break;
                    case METIS_OBJTYPE_VOL:
                        MoveGroupMinConnForVol(
                            ctrl, graph, target, nind, ind, vmarker, pmarker, modind);
                        break;
                    default:
                        gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
                }

                /* Update the csr representation of the partitioning vector */
                iarray2csr(nvtxs, nparts, where, pptr, pind);
            }
        }

        if(move == 0)
            break;
    }

    ipqFree(&queue);

    WCOREPOP;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo */
/*************************************************************************/
void MoveGroupMinConnForCut(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t nind, idx_t* ind)
{
    idx_t      i, ii, j, jj, k, l, nvtxs, nbnd, from, me;
    idx_t *    xadj, *adjncy, *adjwgt, *where, *bndptr, *bndind;
    ckrinfo_t* myrinfo;
    cnbr_t*    mynbrs;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    where  = graph->where;
    bndptr = graph->bndptr;
    bndind = graph->bndind;

    nbnd = graph->nbnd;

    while(--nind >= 0)
    {
        i    = ind[nind];
        from = where[i];

        myrinfo = graph->ckrinfo + i;
        if(myrinfo->inbr == -1)
        {
            myrinfo->inbr  = cnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
            myrinfo->nnbrs = 0;
        }
        mynbrs = ctrl->cnbrpool + myrinfo->inbr;

        /* find the location of 'to' in myrinfo or create it if it is not there */
        for(k = 0; k < myrinfo->nnbrs; k++)
        {
            if(mynbrs[k].pid == to)
                break;
        }
        if(k == myrinfo->nnbrs)
        {
            ASSERT(k < xadj[i + 1] - xadj[i]);
            mynbrs[k].pid = to;
            mynbrs[k].ed  = 0;
            myrinfo->nnbrs++;
        }

        /* Update pwgts */
        iaxpy(graph->ncon,
              1,
              graph->vwgt + i * graph->ncon,
              1,
              graph->pwgts + to * graph->ncon,
              1);
        iaxpy(graph->ncon,
              -1,
              graph->vwgt + i * graph->ncon,
              1,
              graph->pwgts + from * graph->ncon,
              1);

        /* Update mincut */
        graph->mincut -= mynbrs[k].ed - myrinfo->id;

        /* Update subdomain connectivity graph to reflect the move of 'i' */
        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id - mynbrs[k].ed, NULL);

        /* Update ID/ED and BND related information for the moved vertex */
        UpdateMovedVertexInfoAndBND(
            i, from, k, to, myrinfo, mynbrs, where, nbnd, bndptr, bndind, BNDTYPE_REFINE);

        /* Update the degrees of adjacent vertices */
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            ii      = adjncy[j];
            me      = where[ii];
            myrinfo = graph->ckrinfo + ii;

            UpdateAdjacentVertexInfoAndBND(
                ctrl, ii, xadj[ii + 1] - xadj[ii], me, from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, BNDTYPE_REFINE);

            /* Update subdomain graph to reflect the move of 'i' for domains other
         than 'from' and 'to' */
            if(me != from && me != to)
            {
                UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], NULL);
                UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], NULL);
            }
        }
    }

    ASSERT(ComputeCut(graph, where) == graph->mincut);

    graph->nbnd = nbnd;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo */
/*************************************************************************/
void MoveGroupMinConnForVol(ctrl_t*  ctrl,
                            graph_t* graph,
                            idx_t    to,
                            idx_t    nind,
                            idx_t*   ind,
                            idx_t*   vmarker,
                            idx_t*   pmarker,
                            idx_t*   modind)
{
    idx_t      i, ii, j, jj, k, l, nvtxs, from, me, other, xgain, ewgt;
    idx_t *    xadj, *vsize, *adjncy, *where;
    vkrinfo_t *myrinfo, *orinfo;
    vnbr_t *   mynbrs, *onbrs;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vsize  = graph->vsize;
    adjncy = graph->adjncy;
    where  = graph->where;

    while(--nind >= 0)
    {
        i    = ind[nind];
        from = where[i];

        myrinfo = graph->vkrinfo + i;
        if(myrinfo->inbr == -1)
        {
            myrinfo->inbr  = vnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
            myrinfo->nnbrs = 0;
        }
        mynbrs = ctrl->vnbrpool + myrinfo->inbr;

        xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? vsize[i] : 0);

        //printf("Moving %"  PRIDX  " from %"  PRIDX  " to %"  PRIDX  " [vsize: %"  PRIDX  "] [xgain: %"  PRIDX  "]\n",
        //    i, from, to, vsize[i], xgain);

        /* find the location of 'to' in myrinfo or create it if it is not there */
        for(k = 0; k < myrinfo->nnbrs; k++)
        {
            if(mynbrs[k].pid == to)
                break;
        }

        if(k == myrinfo->nnbrs)
        {
            //printf("Missing neighbor\n");

            if(myrinfo->nid > 0)
                xgain -= vsize[i];

            /* determine the volume gain resulting from that move */
            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                ii     = adjncy[j];
                other  = where[ii];
                orinfo = graph->vkrinfo + ii;
                onbrs  = ctrl->vnbrpool + orinfo->inbr;
                ASSERT(other != to)

                //printf("  %8d %8d %3d\n", (int)ii, (int)vsize[ii], (int)other);

                if(from == other)
                {
                    /* Same subdomain vertex: Decrease the gain if 'to' is a new neighbor. */
                    for(l = 0; l < orinfo->nnbrs; l++)
                    {
                        if(onbrs[l].pid == to)
                            break;
                    }
                    if(l == orinfo->nnbrs)
                        xgain -= vsize[ii];
                }
                else
                {
                    /* Remote vertex: increase if 'to' is a new subdomain */
                    for(l = 0; l < orinfo->nnbrs; l++)
                    {
                        if(onbrs[l].pid == to)
                            break;
                    }
                    if(l == orinfo->nnbrs)
                        xgain -= vsize[ii];

                    /* Remote vertex: decrease if i is the only connection to 'from' */
                    for(l = 0; l < orinfo->nnbrs; l++)
                    {
                        if(onbrs[l].pid == from && onbrs[l].ned == 1)
                        {
                            xgain += vsize[ii];
                            break;
                        }
                    }
                }
            }
            graph->minvol -= xgain;
            graph->mincut -= -myrinfo->nid;
            ewgt = myrinfo->nid;
        }
        else
        {
            graph->minvol -= (xgain + mynbrs[k].gv);
            graph->mincut -= mynbrs[k].ned - myrinfo->nid;
            ewgt = myrinfo->nid - mynbrs[k].ned;
        }

        /* Update where and pwgts */
        where[i] = to;
        iaxpy(graph->ncon,
              1,
              graph->vwgt + i * graph->ncon,
              1,
              graph->pwgts + to * graph->ncon,
              1);
        iaxpy(graph->ncon,
              -1,
              graph->vwgt + i * graph->ncon,
              1,
              graph->pwgts + from * graph->ncon,
              1);

        /* Update subdomain connectivity graph to reflect the move of 'i' */
        UpdateEdgeSubDomainGraph(ctrl, from, to, ewgt, NULL);

        /* Update the subdomain connectivity of the adjacent vertices */
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            me = where[adjncy[j]];
            if(me != from && me != to)
            {
                UpdateEdgeSubDomainGraph(ctrl, from, me, -1, NULL);
                UpdateEdgeSubDomainGraph(ctrl, to, me, 1, NULL);
            }
        }

        /* Update the id/ed/gains/bnd of potentially affected nodes */
        KWayVolUpdate(ctrl, graph, i, from, to, NULL, NULL, NULL, NULL, NULL, BNDTYPE_REFINE, vmarker, pmarker, modind);

        /*CheckKWayVolPartitionParams(ctrl, graph);*/
    }
    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERTP(ComputeVolume(graph, where) == graph->minvol,
            ("%" PRIDX " %" PRIDX "\n", ComputeVolume(graph, where), graph->minvol));
}


/*************************************************************************/
/*! This function computes the subdomain graph. For deubugging purposes. */
/*************************************************************************/
void PrintSubDomainGraph(graph_t* graph, idx_t nparts, idx_t* where)
{
    idx_t  i, j, k, me, nvtxs, total, max;
    idx_t *xadj, *adjncy, *adjwgt, *pmat;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    pmat = ismalloc(nparts * nparts, 0, "ComputeSubDomainGraph: pmat");

    for(i = 0; i < nvtxs; i++)
    {
        me = where[i];
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];
            if(where[k] != me)
                pmat[me * nparts + where[k]] += adjwgt[j];
        }
    }

    /* printf("Subdomain Info\n"); */
    total = max = 0;
    for(i = 0; i < nparts; i++)
    {
        for(k = 0, j = 0; j < nparts; j++)
        {
            if(pmat[i * nparts + j] > 0)
                k++;
        }
        total += k;

        if(k > max)
            max = k;
        /*
    printf("%2"  PRIDX  " -> %2"  PRIDX  "  ", i, k);
    for (j=0; j<nparts; j++) {
      if (pmat[i*nparts+j] > 0)
        printf("[%2"  PRIDX  " %4"  PRIDX  "] ", j, pmat[i*nparts+j]);
    }
    printf("\n");
*/
    }
    printf("Total adjacent subdomains: %" PRIDX ", Max: %" PRIDX "\n", total, max);

    gk_free((void**)&pmat, LTERM);
}


/************************ mincover.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * mincover.c
 *
 * This file implements the minimum cover algorithm
 *
 * Started 8/1/97
 * George
 *
 * $Id: mincover.c 9942 2011-05-17 22:09:52Z karypis $
 */


/*************************************************************************
* Constants used by mincover algorithm
**************************************************************************/
#define INCOL 10
#define INROW 20
#define VC 1
#define SC 2
#define HC 3
#define VR 4
#define SR 5
#define HR 6


/*************************************************************************
* This function returns the min-cover of a bipartite graph.
* The algorithm used is due to Hopcroft and Karp as modified by Duff etal
* adj: the adjacency list of the bipartite graph
*       asize: the number of vertices in the first part of the bipartite graph
* bsize-asize: the number of vertices in the second part
*        0..(asize-1) > A vertices
*        asize..bsize > B vertices
*
* Returns:
*  cover : the actual cover (array)
*  csize : the size of the cover
**************************************************************************/
void MinCover(idx_t* xadj, idx_t* adjncy, idx_t asize, idx_t bsize, idx_t* cover, idx_t* csize)
{
    idx_t  i, j;
    idx_t *mate, *queue, *flag, *level, *lst;
    idx_t  fptr, rptr, lstptr;
    idx_t  row, maxlevel, col;

    mate  = ismalloc(bsize, -1, "MinCover: mate");
    flag  = imalloc(bsize, "MinCover: flag");
    level = imalloc(bsize, "MinCover: level");
    queue = imalloc(bsize, "MinCover: queue");
    lst   = imalloc(bsize, "MinCover: lst");

    /* Get a cheap matching */
    for(i = 0; i < asize; i++)
    {
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            if(mate[adjncy[j]] == -1)
            {
                mate[i]         = adjncy[j];
                mate[adjncy[j]] = i;
                break;
            }
        }
    }

    /* Get into the main loop */
    while(1)
    {
        /* Initialization */
        fptr = rptr = 0; /* Empty Queue */
        lstptr      = 0; /* Empty List */
        for(i = 0; i < bsize; i++)
        {
            level[i] = -1;
            flag[i]  = 0;
        }
        maxlevel = bsize;

        /* Insert free nodes into the queue */
        for(i = 0; i < asize; i++)
            if(mate[i] == -1)
            {
                queue[rptr++] = i;
                level[i]      = 0;
            }

        /* Perform the BFS */
        while(fptr != rptr)
        {
            row = queue[fptr++];
            if(level[row] < maxlevel)
            {
                flag[row] = 1;
                for(j = xadj[row]; j < xadj[row + 1]; j++)
                {
                    col = adjncy[j];
                    if(!flag[col])
                    { /* If this column has not been accessed yet */
                        flag[col] = 1;
                        if(mate[col] == -1)
                        { /* Free column node was found */
                            maxlevel      = level[row];
                            lst[lstptr++] = col;
                        }
                        else
                        { /* This column node is matched */
                            if(flag[mate[col]])
                                printf("\nSomething wrong, flag[%" PRIDX "] is 1",
                                       mate[col]);
                            queue[rptr++]    = mate[col];
                            level[mate[col]] = level[row] + 1;
                        }
                    }
                }
            }
        }

        if(lstptr == 0)
            break; /* No free columns can be reached */

        /* Perform restricted DFS from the free column nodes */
        for(i = 0; i < lstptr; i++)
            MinCover_Augment(xadj, adjncy, lst[i], mate, flag, level, maxlevel);
    }

    MinCover_Decompose(xadj, adjncy, asize, bsize, mate, cover, csize);

    gk_free((void**)&mate, &flag, &level, &queue, &lst, LTERM);
}


/*************************************************************************
* This function performs a restricted DFS and augments matchings
**************************************************************************/
idx_t MinCover_Augment(idx_t* xadj, idx_t* adjncy, idx_t col, idx_t* mate, idx_t* flag, idx_t* level, idx_t maxlevel)
{
    idx_t i;
    idx_t row = -1;
    idx_t status;

    flag[col] = 2;
    for(i = xadj[col]; i < xadj[col + 1]; i++)
    {
        row = adjncy[i];

        if(flag[row] == 1)
        { /* First time through this row node */
            if(level[row] == maxlevel)
            {                  /* (col, row) is an edge of the G^T */
                flag[row] = 2; /* Mark this node as being visited */
                if(maxlevel != 0)
                    status = MinCover_Augment(
                        xadj, adjncy, mate[row], mate, flag, level, maxlevel - 1);
                else
                    status = 1;

                if(status)
                {
                    mate[col] = row;
                    mate[row] = col;
                    return 1;
                }
            }
        }
    }

    return 0;
}


/*************************************************************************
* This function performs a coarse decomposition and determines the
* min-cover.
* REF: Pothen ACMTrans. on Amth Software
**************************************************************************/
void MinCover_Decompose(
    idx_t* xadj, idx_t* adjncy, idx_t asize, idx_t bsize, idx_t* mate, idx_t* cover, idx_t* csize)
{
    idx_t  i, k;
    idx_t* where;
    idx_t  card[10];

    where = imalloc(bsize, "MinCover_Decompose: where");
    for(i = 0; i < 10; i++)
        card[i] = 0;

    for(i = 0; i < asize; i++)
        where[i] = SC;
    for(; i < bsize; i++)
        where[i] = SR;

    for(i = 0; i < asize; i++)
        if(mate[i] == -1)
            MinCover_ColDFS(xadj, adjncy, i, mate, where, INCOL);
    for(; i < bsize; i++)
        if(mate[i] == -1)
            MinCover_RowDFS(xadj, adjncy, i, mate, where, INROW);

    for(i = 0; i < bsize; i++)
        card[where[i]]++;

    k = 0;
    if(iabs(card[VC] + card[SC] - card[HR]) < iabs(card[VC] - card[SR] - card[HR]))
    { /* S = VC+SC+HR */
        /* printf("%"  PRIDX  " %"  PRIDX  " ",vc+sc, hr); */
        for(i = 0; i < bsize; i++)
            if(where[i] == VC || where[i] == SC || where[i] == HR)
                cover[k++] = i;
    }
    else
    { /* S = VC+SR+HR */
        /* printf("%"  PRIDX  " %"  PRIDX  " ",vc, hr+sr); */
        for(i = 0; i < bsize; i++)
            if(where[i] == VC || where[i] == SR || where[i] == HR)
                cover[k++] = i;
    }

    *csize = k;
    gk_free((void**)&where, LTERM);
}


/*************************************************************************
* This function performs a dfs starting from an unmatched col node
* forming alternate paths
**************************************************************************/
void MinCover_ColDFS(idx_t* xadj, idx_t* adjncy, idx_t root, idx_t* mate, idx_t* where, idx_t flag)
{
    idx_t i;

    if(flag == INCOL)
    {
        if(where[root] == HC)
            return;
        where[root] = HC;
        for(i = xadj[root]; i < xadj[root + 1]; i++)
            MinCover_ColDFS(xadj, adjncy, adjncy[i], mate, where, INROW);
    }
    else
    {
        if(where[root] == HR)
            return;
        where[root] = HR;
        if(mate[root] != -1)
            MinCover_ColDFS(xadj, adjncy, mate[root], mate, where, INCOL);
    }
}

/*************************************************************************
* This function performs a dfs starting from an unmatched col node
* forming alternate paths
**************************************************************************/
void MinCover_RowDFS(idx_t* xadj, idx_t* adjncy, idx_t root, idx_t* mate, idx_t* where, idx_t flag)
{
    idx_t i;

    if(flag == INROW)
    {
        if(where[root] == VR)
            return;
        where[root] = VR;
        for(i = xadj[root]; i < xadj[root + 1]; i++)
            MinCover_RowDFS(xadj, adjncy, adjncy[i], mate, where, INCOL);
    }
    else
    {
        if(where[root] == VC)
            return;
        where[root] = VC;
        if(mate[root] != -1)
            MinCover_RowDFS(xadj, adjncy, mate[root], mate, where, INROW);
    }
}


/************************ timing.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * timing.c
 *
 * This file contains routines that deal with timing Metis
 *
 * Started 7/24/97
 * George
 *
 * $Id: timing.c 13936 2013-03-30 03:59:09Z karypis $
 *
 */


/*************************************************************************
* This function clears the timers
**************************************************************************/
void InitTimers(ctrl_t* ctrl)
{
    gk_clearcputimer(ctrl->TotalTmr);
    gk_clearcputimer(ctrl->InitPartTmr);
    gk_clearcputimer(ctrl->MatchTmr);
    gk_clearcputimer(ctrl->ContractTmr);
    gk_clearcputimer(ctrl->CoarsenTmr);
    gk_clearcputimer(ctrl->UncoarsenTmr);
    gk_clearcputimer(ctrl->RefTmr);
    gk_clearcputimer(ctrl->ProjectTmr);
    gk_clearcputimer(ctrl->SplitTmr);
    gk_clearcputimer(ctrl->Aux1Tmr);
    gk_clearcputimer(ctrl->Aux2Tmr);
    gk_clearcputimer(ctrl->Aux3Tmr);
}


/*************************************************************************
* This function prints the various timers
**************************************************************************/
void PrintTimers(ctrl_t* ctrl)
{
    printf("\nTiming Information -------------------------------------------------");
    printf("\n Multilevel: \t\t %7.3" PRREAL "", gk_getcputimer(ctrl->TotalTmr));
    printf("\n     Coarsening: \t\t %7.3" PRREAL "", gk_getcputimer(ctrl->CoarsenTmr));
    printf("\n            Matching: \t\t\t %7.3" PRREAL "", gk_getcputimer(ctrl->MatchTmr));
    printf("\n            Contract: \t\t\t %7.3" PRREAL "", gk_getcputimer(ctrl->ContractTmr));
    printf("\n     Initial Partition: \t %7.3" PRREAL "", gk_getcputimer(ctrl->InitPartTmr));
    printf("\n     Uncoarsening: \t\t %7.3" PRREAL "", gk_getcputimer(ctrl->UncoarsenTmr));
    printf("\n          Refinement: \t\t\t %7.3" PRREAL "", gk_getcputimer(ctrl->RefTmr));
    printf("\n          Projection: \t\t\t %7.3" PRREAL "", gk_getcputimer(ctrl->ProjectTmr));
    printf("\n     Splitting: \t\t %7.3" PRREAL "", gk_getcputimer(ctrl->SplitTmr));
    /*
  printf("\n       Aux1Tmr: \t\t %7.3"  PRREAL  "", gk_getcputimer(ctrl->Aux1Tmr));
  printf("\n       Aux2Tmr: \t\t %7.3"  PRREAL  "", gk_getcputimer(ctrl->Aux2Tmr));
  printf("\n       Aux3Tmr: \t\t %7.3"  PRREAL  "", gk_getcputimer(ctrl->Aux3Tmr));
*/
    printf("\n********************************************************************\n");
}


/************************ util.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * util.c
 *
 * This function contains various utility routines
 *
 * Started 9/28/95
 * George
 *
 * $Id: util.c 10495 2011-07-06 16:04:45Z karypis $
 */


/*************************************************************************/
/*! This function initializes the random number generator
  */
/*************************************************************************/
void InitRandom(idx_t seed)
{
    isrand((seed == -1 ? 4321 : seed));
}


/*************************************************************************/
/*! Returns the highest weight index of x[i]*y[i]
 */
/*************************************************************************/
idx_t iargmax_nrm(size_t n, idx_t* x, real_t* y)
{
    idx_t i, max = 0;

    for(i = 1; i < n; i++)
        max = (x[i] * y[i] > x[max] * y[max] ? i : max);

    return max;
}


/*************************************************************************/
/*! These functions return the index of the maximum element in a vector
  */
/*************************************************************************/
idx_t iargmax_strd(size_t n, idx_t* x, idx_t incx)
{
    size_t i, max = 0;

    n *= incx;
    for(i = incx; i < n; i += incx)
        max = (x[i] > x[max] ? i : max);

    return max / incx;
}


/*************************************************************************/
/*! These functions return the index of the almost maximum element in a
    vector
 */
/*************************************************************************/
idx_t rargmax2(size_t n, real_t* x)
{
    size_t i, max1, max2;

    if(x[0] > x[1])
    {
        max1 = 0;
        max2 = 1;
    }
    else
    {
        max1 = 1;
        max2 = 0;
    }

    for(i = 2; i < n; i++)
    {
        if(x[i] > x[max1])
        {
            max2 = max1;
            max1 = i;
        }
        else if(x[i] > x[max2])
            max2 = i;
    }

    return max2;
}


/*************************************************************************/
/*! These functions return the index of the second largest elements in the
    vector formed by x.y where '.' is element-wise multiplication */
/*************************************************************************/
idx_t iargmax2_nrm(size_t n, idx_t* x, real_t* y)
{
    size_t i, max1, max2;

    if(x[0] * y[0] > x[1] * y[1])
    {
        max1 = 0;
        max2 = 1;
    }
    else
    {
        max1 = 1;
        max2 = 0;
    }

    for(i = 2; i < n; i++)
    {
        if(x[i] * y[i] > x[max1] * y[max1])
        {
            max2 = max1;
            max1 = i;
        }
        else if(x[i] * y[i] > x[max2] * y[max2])
            max2 = i;
    }

    return max2;
}


/*************************************************************************/
/*! converts a signal code into a Metis return code
 */
/*************************************************************************/
int metis_rcode(int sigrval)
{
    switch(sigrval)
    {
        case 0:
            return METIS_OK;
            break;
        case SIGMEM:
            return METIS_ERROR_MEMORY;
            break;
        default:
            return METIS_ERROR;
            break;
    }
}


/************************ wspace.c ************************/
/*!
\file
\brief Functions dealing with memory allocation and workspace management

\date Started 2/24/96
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version $Id: wspace.c 10492 2011-07-06 09:28:42Z karypis $
*/


/*************************************************************************/
/*! This function allocates memory for the workspace */
/*************************************************************************/
void AllocateWorkSpace(ctrl_t* ctrl, graph_t* graph)
{
    size_t coresize;

    switch(ctrl->optype)
    {
        case METIS_OP_PMETIS:
            coresize = 3 * (graph->nvtxs + 1) * sizeof(idx_t)
                       + 5 * (ctrl->nparts + 1) * graph->ncon * sizeof(idx_t)
                       + 5 * (ctrl->nparts + 1) * graph->ncon * sizeof(real_t);
            break;
        default:
            coresize = 4 * (graph->nvtxs + 1) * sizeof(idx_t)
                       + 5 * (ctrl->nparts + 1) * graph->ncon * sizeof(idx_t)
                       + 5 * (ctrl->nparts + 1) * graph->ncon * sizeof(real_t);
    }
    ctrl->mcore = gk_mcoreCreate(coresize);

    ctrl->nbrpoolsize = 0;
    ctrl->nbrpoolcpos = 0;
}


/*************************************************************************/
/*! This function allocates refinement-specific memory for the workspace */
/*************************************************************************/
void AllocateRefinementWorkSpace(ctrl_t* ctrl, idx_t nbrpoolsize_max, idx_t nbrpoolsize)
{
    ctrl->nbrpoolsize_max = nbrpoolsize_max;
    ctrl->nbrpoolsize     = nbrpoolsize;
    ctrl->nbrpoolcpos     = 0;
    ctrl->nbrpoolreallocs = 0;

    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT:
            ctrl->cnbrpool = (cnbr_t*)gk_malloc(ctrl->nbrpoolsize * sizeof(cnbr_t),
                                                "AllocateRefinementWorkSpace: cnbrpool");
            break;

        case METIS_OBJTYPE_VOL:
            ctrl->vnbrpool = (vnbr_t*)gk_malloc(ctrl->nbrpoolsize * sizeof(vnbr_t),
                                                "AllocateRefinementWorkSpace: vnbrpool");
            break;

        default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
    }


    /* Allocate the memory for the sparse subdomain graph */
    if(ctrl->minconn)
    {
        ctrl->pvec1 = imalloc(ctrl->nparts + 1, "AllocateRefinementWorkSpace: pvec1");
        ctrl->pvec2 = imalloc(ctrl->nparts + 1, "AllocateRefinementWorkSpace: pvec2");
        ctrl->maxnads = ismalloc(ctrl->nparts, INIT_MAXNAD, "AllocateRefinementWorkSpace: maxnads");
        ctrl->nads = imalloc(ctrl->nparts, "AllocateRefinementWorkSpace: nads");
        ctrl->adids = iAllocMatrix(ctrl->nparts, INIT_MAXNAD, 0, "AllocateRefinementWorkSpace: adids");
        ctrl->adwgts = iAllocMatrix(ctrl->nparts, INIT_MAXNAD, 0, "AllocateRefinementWorkSpace: adwgts");
    }
}


/*************************************************************************/
/*! This function frees the workspace */
/*************************************************************************/
void FreeWorkSpace(ctrl_t* ctrl)
{
    gk_mcoreDestroy(&ctrl->mcore, ctrl->dbglvl & METIS_DBG_INFO);

    IFSET(ctrl->dbglvl,
          METIS_DBG_INFO,
          printf(" nbrpool statistics\n"
                 "        nbrpoolsize: %12zu   nbrpoolcpos: %12zu\n"
                 "    nbrpoolreallocs: %12zu\n\n",
                 ctrl->nbrpoolsize,
                 ctrl->nbrpoolcpos,
                 ctrl->nbrpoolreallocs));

    gk_free((void**)&ctrl->cnbrpool, &ctrl->vnbrpool, LTERM);
    ctrl->nbrpoolsize_max = 0;
    ctrl->nbrpoolsize     = 0;
    ctrl->nbrpoolcpos     = 0;

    if(ctrl->minconn)
    {
        iFreeMatrix(&(ctrl->adids), ctrl->nparts, INIT_MAXNAD);
        iFreeMatrix(&(ctrl->adwgts), ctrl->nparts, INIT_MAXNAD);

        gk_free((void**)&ctrl->pvec1, &ctrl->pvec2, &ctrl->maxnads, &ctrl->nads, LTERM);
    }
}


/*************************************************************************/
/*! This function allocate space from the workspace/heap */
/*************************************************************************/
void* wspacemalloc(ctrl_t* ctrl, size_t nbytes)
{
    return gk_mcoreMalloc(ctrl->mcore, nbytes);
}


/*************************************************************************/
/*! This function sets a marker in the stack of malloc ops to be used
    subsequently for freeing purposes */
/*************************************************************************/
void wspacepush(ctrl_t* ctrl)
{
    gk_mcorePush(ctrl->mcore);
}


/*************************************************************************/
/*! This function frees all mops since the last push */
/*************************************************************************/
void wspacepop(ctrl_t* ctrl)
{
    gk_mcorePop(ctrl->mcore);
}


/*************************************************************************/
/*! This function allocate space from the core  */
/*************************************************************************/
idx_t* iwspacemalloc(ctrl_t* ctrl, idx_t n)
{
    return (idx_t*)wspacemalloc(ctrl, n * sizeof(idx_t));
}


/*************************************************************************/
/*! This function allocate space from the core */
/*************************************************************************/
real_t* rwspacemalloc(ctrl_t* ctrl, idx_t n)
{
    return (real_t*)wspacemalloc(ctrl, n * sizeof(real_t));
}


/*************************************************************************/
/*! This function allocate space from the core  */
/*************************************************************************/
ikv_t* ikvwspacemalloc(ctrl_t* ctrl, idx_t n)
{
    return (ikv_t*)wspacemalloc(ctrl, n * sizeof(ikv_t));
}


/*************************************************************************/
/*! This function resets the cnbrpool */
/*************************************************************************/
void cnbrpoolReset(ctrl_t* ctrl)
{
    ctrl->nbrpoolcpos = 0;
}


/*************************************************************************/
/*! This function gets the next free index from cnbrpool */
/*************************************************************************/
idx_t cnbrpoolGetNext(ctrl_t* ctrl, idx_t nnbrs)
{
    nnbrs = gk_min(ctrl->nparts, nnbrs);
    ctrl->nbrpoolcpos += nnbrs;

    if(ctrl->nbrpoolcpos > ctrl->nbrpoolsize)
    {
        ctrl->nbrpoolsize += gk_max(10 * nnbrs, ctrl->nbrpoolsize / 2);
        ctrl->nbrpoolsize = gk_min(ctrl->nbrpoolsize, ctrl->nbrpoolsize_max);

        ctrl->cnbrpool = (cnbr_t*)gk_realloc(ctrl->cnbrpool,
                                             ctrl->nbrpoolsize * sizeof(cnbr_t),
                                             "cnbrpoolGet: cnbrpool");
        ctrl->nbrpoolreallocs++;
    }

    return ctrl->nbrpoolcpos - nnbrs;
}


/*************************************************************************/
/*! This function resets the vnbrpool */
/*************************************************************************/
void vnbrpoolReset(ctrl_t* ctrl)
{
    ctrl->nbrpoolcpos = 0;
}


/*************************************************************************/
/*! This function gets the next free index from vnbrpool */
/*************************************************************************/
idx_t vnbrpoolGetNext(ctrl_t* ctrl, idx_t nnbrs)
{
    nnbrs = gk_min(ctrl->nparts, nnbrs);
    ctrl->nbrpoolcpos += nnbrs;

    if(ctrl->nbrpoolcpos > ctrl->nbrpoolsize)
    {
        ctrl->nbrpoolsize += gk_max(10 * nnbrs, ctrl->nbrpoolsize / 2);
        ctrl->nbrpoolsize = gk_min(ctrl->nbrpoolsize, ctrl->nbrpoolsize_max);

        ctrl->vnbrpool = (vnbr_t*)gk_realloc(ctrl->vnbrpool,
                                             ctrl->nbrpoolsize * sizeof(vnbr_t),
                                             "vnbrpoolGet: vnbrpool");
        ctrl->nbrpoolreallocs++;
    }

    return ctrl->nbrpoolcpos - nnbrs;
}
