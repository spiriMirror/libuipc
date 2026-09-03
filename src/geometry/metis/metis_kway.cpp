/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 * This file is split from the port's merged implementation.
 */

#include <metis.h>

/* The merged implementation removes unused source files and GK_MK* macro
 * instantiations while preserving the METIS_PartGraphKway algorithm.
 */

/************************ fm.c ************************/
/*!
\file
\brief Functions for the edge-based FM refinement

\date Started 7/23/97
\author George
\author Copyright 1997-2011, Regents of the University of Minnesota
\version\verbatim $Id: fm.c 10187 2011-06-13 13:46:57Z karypis $ \endverbatim
*/


/*************************************************************************
* This function performs an edge-based FM refinement
**************************************************************************/
void FM_2WayRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter)
{
    if(graph->ncon == 1)
        FM_2WayCutRefine(ctrl, graph, ntpwgts, niter);
    else
        FM_Mc2WayCutRefine(ctrl, graph, ntpwgts, niter);
}


/*************************************************************************/
/*! This function performs a cut-focused FM refinement */
/*************************************************************************/
void FM_2WayCutRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter)
{
    idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, me, limit, tmp;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *id, *ed, *bndptr, *bndind, *pwgts;
    idx_t *moved, *swaps, *perm;
    rpq_t* queues[2];
    idx_t higain, mincut, mindiff, origdiff, initcut, newcut, mincutorder, avgvwgt;
    idx_t tpwgts[2];

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
    swaps = iwspacemalloc(ctrl, nvtxs);
    perm  = iwspacemalloc(ctrl, nvtxs);

    tpwgts[0] = graph->tvwgt[0] * ntpwgts[0];
    tpwgts[1] = graph->tvwgt[0] - tpwgts[0];

    limit = gk_min(gk_max(0.01 * nvtxs, 15), 100);
    avgvwgt = gk_min((pwgts[0] + pwgts[1]) / 20, 2 * (pwgts[0] + pwgts[1]) / nvtxs);

    queues[0] = rpqCreate(nvtxs);
    queues[1] = rpqCreate(nvtxs);

    IFSET(ctrl->dbglvl, METIS_DBG_REFINE, Print2WayRefineStats(ctrl, graph, ntpwgts, 0, -2));

    origdiff = iabs(tpwgts[0] - pwgts[0]);
    iset(nvtxs, -1, moved);
    for(pass = 0; pass < niter; pass++)
    { /* Do a number of passes */
        rpqReset(queues[0]);
        rpqReset(queues[1]);

        mincutorder = -1;
        newcut = mincut = initcut = graph->mincut;
        mindiff                   = iabs(tpwgts[0] - pwgts[0]);

        ASSERT(ComputeCut(graph, where) == graph->mincut);
        ASSERT(CheckBnd(graph));

        /* Insert boundary nodes in the priority queues */
        nbnd = graph->nbnd;
        irandArrayPermute(nbnd, perm, nbnd, 1);
        for(ii = 0; ii < nbnd; ii++)
        {
            i = perm[ii];
            ASSERT(ed[bndind[i]] > 0 || id[bndind[i]] == 0);
            ASSERT(bndptr[bndind[i]] != -1);
            rpqInsert(queues[where[bndind[i]]], bndind[i], ed[bndind[i]] - id[bndind[i]]);
        }

        for(nswaps = 0; nswaps < nvtxs; nswaps++)
        {
            from = (tpwgts[0] - pwgts[0] < tpwgts[1] - pwgts[1] ? 0 : 1);
            to   = (from + 1) % 2;

            if((higain = rpqGetTop(queues[from])) == -1)
                break;
            ASSERT(bndptr[higain] != -1);

            newcut -= (ed[higain] - id[higain]);
            INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

            if((newcut < mincut && iabs(tpwgts[0] - pwgts[0]) <= origdiff + avgvwgt)
               || (newcut == mincut && iabs(tpwgts[0] - pwgts[0]) < mindiff))
            {
                mincut      = newcut;
                mindiff     = iabs(tpwgts[0] - pwgts[0]);
                mincutorder = nswaps;
            }
            else if(nswaps - mincutorder > limit)
            { /* We hit the limit, undo last move */
                newcut += (ed[higain] - id[higain]);
                INC_DEC(pwgts[from], pwgts[to], vwgt[higain]);
                break;
            }

            where[higain] = to;
            moved[higain] = nswaps;
            swaps[nswaps] = higain;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("Moved %6" PRIDX " from %" PRIDX ". [%3" PRIDX
                         " %3" PRIDX "] %5" PRIDX " [%4" PRIDX " %4" PRIDX "]\n",
                         higain,
                         from,
                         ed[higain] - id[higain],
                         vwgt[higain],
                         newcut,
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
                k = adjncy[j];

                kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                INC_DEC(id[k], ed[k], kwgt);

                /* Update its boundary information and queue position */
                if(bndptr[k] != -1)
                { /* If k was a boundary vertex */
                    if(ed[k] == 0)
                    { /* Not a boundary vertex any more */
                        BNDDelete(nbnd, bndind, bndptr, k);
                        if(moved[k] == -1) /* Remove it if in the queues */
                            rpqDelete(queues[where[k]], k);
                    }
                    else
                    { /* If it has not been moved, update its position in the queue */
                        if(moved[k] == -1)
                            rpqUpdate(queues[where[k]], k, ed[k] - id[k]);
                    }
                }
                else
                {
                    if(ed[k] > 0)
                    { /* It will now become a boundary vertex */
                        BNDInsert(nbnd, bndind, bndptr, k);
                        if(moved[k] == -1)
                            rpqInsert(queues[where[k]], k, ed[k] - id[k]);
                    }
                }
            }
        }


        /****************************************************************
    * Roll back computations
    *****************************************************************/
        for(i = 0; i < nswaps; i++)
            moved[swaps[i]] = -1; /* reset moved array */
        for(nswaps--; nswaps > mincutorder; nswaps--)
        {
            higain = swaps[nswaps];

            to = where[higain] = (where[higain] + 1) % 2;
            SWAP(id[higain], ed[higain], tmp);
            if(ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
                BNDDelete(nbnd, bndind, bndptr, higain);
            else if(ed[higain] > 0 && bndptr[higain] == -1)
                BNDInsert(nbnd, bndind, bndptr, higain);

            INC_DEC(pwgts[to], pwgts[(to + 1) % 2], vwgt[higain]);
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

        graph->mincut = mincut;
        graph->nbnd   = nbnd;

        IFSET(ctrl->dbglvl,
              METIS_DBG_REFINE,
              Print2WayRefineStats(ctrl, graph, ntpwgts, 0, mincutorder));

        if(mincutorder <= 0 || mincut == initcut)
            break;
    }

    rpqDestroy(queues[0]);
    rpqDestroy(queues[1]);

    WCOREPOP;
}


/*************************************************************************/
/*! This function performs a cut-focused multi-constraint FM refinement */
/*************************************************************************/
void FM_Mc2WayCutRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter)
{
    idx_t i, ii, j, k, l, kwgt, nvtxs, ncon, nbnd, nswaps, from, to, pass, me,
        limit, tmp, cnum;
    idx_t *xadj, *adjncy, *vwgt, *adjwgt, *pwgts, *where, *id, *ed, *bndptr, *bndind;
    idx_t * moved, *swaps, *perm, *qnum;
    idx_t   higain, mincut, initcut, newcut, mincutorder;
    real_t *invtvwgt, *ubfactors, *minbalv, *newbalv;
    real_t  origbal, minbal, newbal, rgain, ffactor;
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

    moved     = iwspacemalloc(ctrl, nvtxs);
    swaps     = iwspacemalloc(ctrl, nvtxs);
    perm      = iwspacemalloc(ctrl, nvtxs);
    qnum      = iwspacemalloc(ctrl, nvtxs);
    ubfactors = rwspacemalloc(ctrl, ncon);
    newbalv   = rwspacemalloc(ctrl, ncon);
    minbalv   = rwspacemalloc(ctrl, ncon);

    limit = gk_min(gk_max(0.01 * nvtxs, 25), 150);


    /* Determine a fudge factor to allow the refinement routines to get out
     of tight balancing constraints. */
    ffactor = .5 / gk_max(20, nvtxs);

    /* Initialize the queues */
    queues = (rpq_t**)wspacemalloc(ctrl, 2 * ncon * sizeof(rpq_t*));
    for(i = 0; i < 2 * ncon; i++)
        queues[i] = rpqCreate(nvtxs);
    for(i = 0; i < nvtxs; i++)
        qnum[i] = iargmax_nrm(ncon, vwgt + i * ncon, invtvwgt);

    /* Determine the unbalance tolerance for each constraint. The tolerance is
     equal to the maximum of the original load imbalance and the user-supplied
     allowed tolerance. The rationale behind this approach is to allow the
     refinement routine to improve the cut, without having to worry about fixing
     load imbalance problems. The load imbalance is addressed by the balancing
     routines. */
    origbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ctrl->ubfactors, ubfactors);
    for(i = 0; i < ncon; i++)
        ubfactors[i] = (ubfactors[i] > 0 ? ctrl->ubfactors[i] + ubfactors[i] :
                                           ctrl->ubfactors[i]);


    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          Print2WayRefineStats(ctrl, graph, ntpwgts, origbal, -2));

    iset(nvtxs, -1, moved);
    for(pass = 0; pass < niter; pass++)
    { /* Do a number of passes */
        for(i = 0; i < 2 * ncon; i++)
            rpqReset(queues[i]);

        mincutorder = -1;
        newcut = mincut = initcut = graph->mincut;

        minbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ubfactors, minbalv);

        ASSERT(ComputeCut(graph, where) == graph->mincut);
        ASSERT(CheckBnd(graph));

        /* Insert boundary nodes in the priority queues */
        nbnd = graph->nbnd;
        irandArrayPermute(nbnd, perm, nbnd / 5, 1);
        for(ii = 0; ii < nbnd; ii++)
        {
            i = bndind[perm[ii]];
            ASSERT(ed[i] > 0 || id[i] == 0);
            ASSERT(bndptr[i] != -1);
            //rgain = 1.0*(ed[i]-id[i])/sqrt(vwgt[i*ncon+qnum[i]]+1);
            //rgain = (ed[i]-id[i] > 0 ? 1.0*(ed[i]-id[i])/sqrt(vwgt[i*ncon+qnum[i]]+1) : ed[i]-id[i]);
            rgain = ed[i] - id[i];
            rpqInsert(queues[2 * qnum[i] + where[i]], i, rgain);
        }

        for(nswaps = 0; nswaps < nvtxs; nswaps++)
        {
            SelectQueue(graph, ctrl->pijbm, ubfactors, queues, &from, &cnum);

            to = (from + 1) % 2;

            if(from == -1 || (higain = rpqGetTop(queues[2 * cnum + from])) == -1)
                break;
            ASSERT(bndptr[higain] != -1);

            newcut -= (ed[higain] - id[higain]);

            iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
            iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
            newbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ubfactors, newbalv);

            if((newcut < mincut && newbal <= ffactor)
               || (newcut == mincut
                   && (newbal < minbal
                       || (newbal == minbal && BetterBalance2Way(ncon, minbalv, newbalv)))))
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
                printf("Moved%6" PRIDX " from %" PRIDX "(%" PRIDX ") Gain:%5" PRIDX
                       ", "
                       "Cut:%5" PRIDX ", NPwgts:",
                       higain,
                       from,
                       cnum,
                       ed[higain] - id[higain],
                       newcut);
                for(l = 0; l < ncon; l++)
                    printf("(%.3" PRREAL " %.3" PRREAL ")",
                           pwgts[l] * invtvwgt[l],
                           pwgts[ncon + l] * invtvwgt[l]);
                printf(" %+.3" PRREAL " LB: %.3" PRREAL "(%+.3" PRREAL ")\n",
                       minbal,
                       ComputeLoadImbalance(graph, 2, ctrl->pijbm),
                       newbal);
            }


            /**************************************************************
      * Update the id[i]/ed[i] values of the affected nodes
      ***************************************************************/
            SWAP(id[higain], ed[higain], tmp);
            if(ed[higain] == 0 && xadj[higain] < xadj[higain + 1])
                BNDDelete(nbnd, bndind, bndptr, higain);

            for(j = xadj[higain]; j < xadj[higain + 1]; j++)
            {
                k = adjncy[j];

                kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                INC_DEC(id[k], ed[k], kwgt);

                /* Update its boundary information and queue position */
                if(bndptr[k] != -1)
                { /* If k was a boundary vertex */
                    if(ed[k] == 0)
                    { /* Not a boundary vertex any more */
                        BNDDelete(nbnd, bndind, bndptr, k);
                        if(moved[k] == -1) /* Remove it if in the queues */
                            rpqDelete(queues[2 * qnum[k] + where[k]], k);
                    }
                    else
                    { /* If it has not been moved, update its position in the queue */
                        if(moved[k] == -1)
                        {
                            //rgain = 1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1);
                            //rgain = (ed[k]-id[k] > 0 ?
                            //              1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1) : ed[k]-id[k]);
                            rgain = ed[k] - id[k];
                            rpqUpdate(queues[2 * qnum[k] + where[k]], k, rgain);
                        }
                    }
                }
                else
                {
                    if(ed[k] > 0)
                    { /* It will now become a boundary vertex */
                        BNDInsert(nbnd, bndind, bndptr, k);
                        if(moved[k] == -1)
                        {
                            //rgain = 1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1);
                            //rgain = (ed[k]-id[k] > 0 ?
                            //              1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1) : ed[k]-id[k]);
                            rgain = ed[k] - id[k];
                            rpqInsert(queues[2 * qnum[k] + where[k]], k, rgain);
                        }
                    }
                }
            }
        }


        /****************************************************************
    * Roll back computations
    *****************************************************************/
        for(i = 0; i < nswaps; i++)
            moved[swaps[i]] = -1; /* reset moved array */
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

        graph->mincut = mincut;
        graph->nbnd   = nbnd;

        IFSET(ctrl->dbglvl,
              METIS_DBG_REFINE,
              Print2WayRefineStats(ctrl, graph, ntpwgts, minbal, mincutorder));

        if(mincutorder <= 0 || mincut == initcut)
            break;
    }

    for(i = 0; i < 2 * ncon; i++)
        rpqDestroy(queues[i]);

    WCOREPOP;
}


/*************************************************************************/
/*! This function selects the partition number and the queue from which
    we will move vertices out. */
/*************************************************************************/
void SelectQueue(graph_t* graph, real_t* pijbm, real_t* ubfactors, rpq_t** queues, idx_t* from, idx_t* cnum)
{
    idx_t  ncon, i, part;
    real_t max, tmp;

    ncon = graph->ncon;

    *from = -1;
    *cnum = -1;

    /* First determine the side and the queue, irrespective of the presence of nodes.
     The side & queue is determined based on the most violated balancing constraint. */
    for(max = 0.0, part = 0; part < 2; part++)
    {
        for(i = 0; i < ncon; i++)
        {
            tmp = graph->pwgts[part * ncon + i] * pijbm[part * ncon + i] - ubfactors[i];
            /* the '=' in the test below is to ensure that under tight constraints
         the partition that is at the max is selected */
            if(tmp >= max)
            {
                max   = tmp;
                *from = part;
                *cnum = i;
            }
        }
    }


    if(*from != -1)
    {
        /* in case the desired queue is empty, select a queue from the same side */
        if(rpqLength(queues[2 * (*cnum) + (*from)]) == 0)
        {
            for(i = 0; i < ncon; i++)
            {
                if(rpqLength(queues[2 * i + (*from)]) > 0)
                {
                    max = graph->pwgts[(*from) * ncon + i] * pijbm[(*from) * ncon + i]
                          - ubfactors[i];
                    *cnum = i;
                    break;
                }
            }

            for(i++; i < ncon; i++)
            {
                tmp = graph->pwgts[(*from) * ncon + i] * pijbm[(*from) * ncon + i]
                      - ubfactors[i];
                if(tmp > max && rpqLength(queues[2 * i + (*from)]) > 0)
                {
                    max   = tmp;
                    *cnum = i;
                }
            }
        }

        /*
    printf("Selected1 %"  PRIDX  "(%"  PRIDX  ") -> %"  PRIDX  " [%5"  PRREAL  "]\n",
        *from, *cnum, rpqLength(queues[2*(*cnum)+(*from)]), max);
    */
    }
    else
    {
        /* the partitioning does not violate balancing constraints, in which case select
       a queue based on cut criteria */
        for(part = 0; part < 2; part++)
        {
            for(i = 0; i < ncon; i++)
            {
                if(rpqLength(queues[2 * i + part]) > 0
                   && (*from == -1 || rpqSeeTopKey(queues[2 * i + part]) > max))
                {
                    max   = rpqSeeTopKey(queues[2 * i + part]);
                    *from = part;
                    *cnum = i;
                }
            }
        }
        /*
    printf("Selected2 %"  PRIDX  "(%"  PRIDX  ") -> %"  PRIDX  "\n",
        *from, *cnum, rpqLength(queues[2*(*cnum)+(*from)]), max);
    */
    }
}


/*************************************************************************/
/*! Prints statistics about the refinement */
/*************************************************************************/
void Print2WayRefineStats(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, real_t deltabal, idx_t mincutorder)
{
    int i;

    if(mincutorder == -2)
    {
        printf("Parts: ");
        printf("Nv-Nb[%5" PRIDX " %5" PRIDX "] ICut: %6" PRIDX,
               graph->nvtxs,
               graph->nbnd,
               graph->mincut);
        printf(" [");
        for(i = 0; i < graph->ncon; i++)
            printf("(%.3" PRREAL " %.3" PRREAL " T:%.3" PRREAL " %.3" PRREAL ")",
                   graph->pwgts[i] * graph->invtvwgt[i],
                   graph->pwgts[graph->ncon + i] * graph->invtvwgt[i],
                   ntpwgts[i],
                   ntpwgts[graph->ncon + i]);
        printf("] LB: %.3" PRREAL "(%+.3" PRREAL ")\n",
               ComputeLoadImbalance(graph, 2, ctrl->pijbm),
               deltabal);
    }
    else
    {
        printf("\tMincut: %6" PRIDX " at %5" PRIDX " NBND %6" PRIDX " NPwgts: [",
               graph->mincut,
               mincutorder,
               graph->nbnd);
        for(i = 0; i < graph->ncon; i++)
            printf("(%.3" PRREAL " %.3" PRREAL ")",
                   graph->pwgts[i] * graph->invtvwgt[i],
                   graph->pwgts[graph->ncon + i] * graph->invtvwgt[i]);
        printf("] LB: %.3" PRREAL "(%+.3" PRREAL ")\n",
               ComputeLoadImbalance(graph, 2, ctrl->pijbm),
               deltabal);
    }
}

/************************ kwayfm.c ************************/
/*!
\file
\brief Routines for k-way refinement

\date Started 7/28/97
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version $Id: kwayfm.c 17513 2014-08-05 16:20:50Z dominique $
*/


/*************************************************************************/
/* Top-level routine for k-way partitioning refinement. This routine just
   calls the appropriate refinement routine based on the objectives and
   constraints. */
/*************************************************************************/
void Greedy_KWayOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode)
{
    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT:
            if(graph->ncon == 1)
                Greedy_KWayCutOptimize(ctrl, graph, niter, ffactor, omode);
            else
                Greedy_McKWayCutOptimize(ctrl, graph, niter, ffactor, omode);
            break;

        case METIS_OBJTYPE_VOL:
            if(graph->ncon == 1)
                Greedy_KWayVolOptimize(ctrl, graph, niter, ffactor, omode);
            else
                Greedy_McKWayVolOptimize(ctrl, graph, niter, ffactor, omode);
            break;

        default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
    }
}


/*************************************************************************/
/*! K-way partitioning optimization in which the vertices are visited in
    decreasing ed/sqrt(nnbrs)-id order. Note this is just an
    approximation, as the ed is often split across different subdomains
    and the sqrt(nnbrs) is just a crude approximation.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves
         to violate the max-pwgt constraint.
  \param omode is the type of optimization that will performed among
         OMODE_REFINE and OMODE_BALANCE

*/
/**************************************************************************/
void Greedy_KWayCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode)
{
    /* Common variables to all types of kway-refinement/balancing routines */
    idx_t  i, ii, iii, j, k, l, pass, nvtxs, nparts, gain;
    idx_t  from, me, to, oldcut, vwgt;
    idx_t *xadj, *adjncy, *adjwgt;
    idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minpwgts, *maxpwgts;
    idx_t  nmoved, nupd, *vstatus, *updptr, *updind;
    idx_t maxndoms, *safetos = NULL, *nads = NULL, *doms = NULL, **adids = NULL,
                    **adwgts = NULL;
    idx_t *bfslvl = NULL, *bfsind = NULL, *bfsmrk = NULL;
    idx_t  bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
    real_t *tpwgts, ubfactor;

    /* Edgecut-specific/different variables */
    idx_t      nbnd, oldnnbrs;
    rpq_t*     queue;
    real_t     rgain;
    ckrinfo_t* myrinfo;
    cnbr_t*    mynbrs;

    ffactor = 0.0;
    WCOREPUSH;

    /* Link the graph fields */
    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;

    where = graph->where;
    pwgts = graph->pwgts;

    nparts = ctrl->nparts;
    tpwgts = ctrl->tpwgts;

    /* Setup the weight intervals of the various subdomains */
    minpwgts = iwspacemalloc(ctrl, nparts);
    maxpwgts = iwspacemalloc(ctrl, nparts);

    if(omode == OMODE_BALANCE)
        ubfactor = ctrl->ubfactors[0];
    else
        ubfactor = gk_max(ctrl->ubfactors[0],
                          ComputeLoadImbalance(graph, nparts, ctrl->pijbm));

    for(i = 0; i < nparts; i++)
    {
        maxpwgts[i] = tpwgts[i] * graph->tvwgt[0] * ubfactor;
        minpwgts[i] = tpwgts[i] * graph->tvwgt[0] * (1.0 / ubfactor);
    }

    perm = iwspacemalloc(ctrl, nvtxs);


    /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made.
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
    safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

    if(ctrl->minconn)
    {
        ComputeSubDomainGraph(ctrl, graph);

        nads   = ctrl->nads;
        adids  = ctrl->adids;
        adwgts = ctrl->adwgts;
        doms   = iset(nparts, 0, ctrl->pvec1);
    }


    /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
    vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
    updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
    updind  = iwspacemalloc(ctrl, nvtxs);

    if(ctrl->contig)
    {
        /* The arrays that will be used for limited check of articulation points */
        bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
        bfsind = iwspacemalloc(ctrl, nvtxs);
        bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    }

    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("%s: [%6" PRIDX " %6" PRIDX "]-[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL
               ","
               " Nv-Nb[%6" PRIDX " %6" PRIDX "], Cut: %6" PRIDX,
               (omode == OMODE_REFINE ? "GRC" : "GBC"),
               pwgts[iargmin(nparts, pwgts, 1)],
               imax(nparts, pwgts, 1),
               minpwgts[0],
               maxpwgts[0],
               ComputeLoadImbalance(graph, nparts, ctrl->pijbm),
               graph->nvtxs,
               graph->nbnd,
               graph->mincut);
        if(ctrl->minconn)
            printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                   imax(nparts, nads, 1),
                   isum(nparts, nads, 1));
        printf("\n");
    }

    queue = rpqCreate(nvtxs);

    /*=====================================================================
  * The top-level refinement loop
  *======================================================================*/
    for(pass = 0; pass < niter; pass++)
    {
        ASSERT(ComputeCut(graph, where) == graph->mincut);
        if(omode == OMODE_REFINE)
            ASSERT(CheckBnd2(graph));

        if(omode == OMODE_BALANCE)
        {
            /* Check to see if things are out of balance, given the tolerance */
            for(i = 0; i < nparts; i++)
            {
                if(pwgts[i] > maxpwgts[i] || pwgts[i] < minpwgts[i])
                    break;
            }
            if(i == nparts) /* Things are balanced. Return right away */
                break;
        }

        oldcut = graph->mincut;
        nbnd   = graph->nbnd;
        nupd   = 0;

        if(ctrl->minconn)
            maxndoms = imax(nparts, nads, 1);

        /* Insert the boundary vertices in the priority queue */
        irandArrayPermute(nbnd, perm, nbnd / 4, 1);
        for(ii = 0; ii < nbnd; ii++)
        {
            i     = bndind[perm[ii]];
            rgain = (graph->ckrinfo[i].nnbrs > 0 ?
                         1.0 * graph->ckrinfo[i].ed / sqrt(graph->ckrinfo[i].nnbrs) :
                         0.0)
                    - graph->ckrinfo[i].id;
            rpqInsert(queue, i, rgain);
            vstatus[i] = VPQSTATUS_PRESENT;
            ListInsert(nupd, updind, updptr, i);
        }

        /* Start extracting vertices from the queue and try to move them */
        for(nmoved = 0, iii = 0;; iii++)
        {
            if((i = rpqGetTop(queue)) == -1)
                break;
            vstatus[i] = VPQSTATUS_EXTRACTED;

            myrinfo = graph->ckrinfo + i;
            mynbrs  = ctrl->cnbrpool + myrinfo->inbr;

            from = where[i];
            vwgt = graph->vwgt[i];

#ifdef XXX
            /* Prevent moves that make 'from' domain underbalanced */
            if(omode == OMODE_REFINE)
            {
                if(myrinfo->id > 0 && pwgts[from] - vwgt < minpwgts[from])
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                if(pwgts[from] - vwgt < minpwgts[from])
                    continue;
            }
#endif

            if(ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                continue;

            if(ctrl->minconn)
                SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

            /* Find the most promising subdomain to move to */
            if(omode == OMODE_REFINE)
            {
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    if(((mynbrs[k].ed > myrinfo->id)
                        && ((pwgts[from] - vwgt >= minpwgts[from])
                            || (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt)))
                        && ((pwgts[to] + vwgt <= maxpwgts[to])
                            || (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))))
                       || ((mynbrs[k].ed == myrinfo->id)
                           && (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))))
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    if(((mynbrs[j].ed > mynbrs[k].ed)
                        && ((pwgts[from] - vwgt >= minpwgts[from])
                            || (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt)))
                        && ((pwgts[to] + vwgt <= maxpwgts[to])
                            || (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))))
                       || ((mynbrs[j].ed == mynbrs[k].ed)
                           && (tpwgts[mynbrs[k].pid] * pwgts[to]
                               < tpwgts[to] * pwgts[mynbrs[k].pid])))
                        k = j;
                }

                to = mynbrs[k].pid;

                gain = mynbrs[k].ed - myrinfo->id;
                /*
        if (!(gain > 0
              || (gain == 0
                  && (pwgts[from] >= maxpwgts[from]
                      || tpwgts[to]*pwgts[from] > tpwgts[from]*(pwgts[to]+vwgt)
                      || (iii%2 == 0 && safetos[to] == 2)
                     )
                 )
             )
           )
          continue;
        */
            }
            else
            { /* OMODE_BALANCE */
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    /* the correctness of the following test follows from the correctness
             of the similar test in the subsequent loop */
                    if(from >= nparts
                       || tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    if(tpwgts[mynbrs[k].pid] * pwgts[to]
                       < tpwgts[to] * pwgts[mynbrs[k].pid])
                        k = j;
                }

                to = mynbrs[k].pid;

                //if (pwgts[from] < maxpwgts[from] && pwgts[to] > minpwgts[to] &&
                //    mynbrs[k].ed-myrinfo->id < 0)
                //  continue;
            }


            /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to'
      *======================================================================*/
            graph->mincut -= mynbrs[k].ed - myrinfo->id;
            nmoved++;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("\t\tMoving %6" PRIDX " from %3" PRIDX "/%" PRIDX
                         " to %3" PRIDX "/%" PRIDX " [%6" PRIDX " %6" PRIDX
                         "]. Gain: %4" PRIDX ". Cut: %6" PRIDX "\n",
                         i,
                         from,
                         safetos[from],
                         to,
                         safetos[to],
                         pwgts[from],
                         pwgts[to],
                         mynbrs[k].ed - myrinfo->id,
                         graph->mincut));

            /* Update the subdomain connectivity information */
            if(ctrl->minconn)
            {
                /* take care of i's move itself */
                UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id - mynbrs[k].ed, &maxndoms);

                /* take care of the adjacent vertices */
                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    me = where[adjncy[j]];
                    if(me != from && me != to)
                    {
                        UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], &maxndoms);
                        UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], &maxndoms);
                    }
                }
            }

            /* Update ID/ED and BND related information for the moved vertex */
            INC_DEC(pwgts[to], pwgts[from], vwgt);
            UpdateMovedVertexInfoAndBND(
                i, from, k, to, myrinfo, mynbrs, where, nbnd, bndptr, bndind, bndtype);

            /* Update the degrees of adjacent vertices */
            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                ii      = adjncy[j];
                me      = where[ii];
                myrinfo = graph->ckrinfo + ii;

                oldnnbrs = myrinfo->nnbrs;

                UpdateAdjacentVertexInfoAndBND(
                    ctrl, ii, xadj[ii + 1] - xadj[ii], me, from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

                UpdateQueueInfo(
                    queue, vstatus, ii, me, from, to, myrinfo, oldnnbrs, nupd, updptr, updind, bndtype);

                ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
            }
        }

        graph->nbnd = nbnd;

        /* Reset the vstatus and associated data structures */
        for(i = 0; i < nupd; i++)
        {
            ASSERT(updptr[updind[i]] != -1);
            ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
            vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
            updptr[updind[i]]  = -1;
        }

        if(ctrl->dbglvl & METIS_DBG_REFINE)
        {
            printf("\t[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL ", Nb: %6" PRIDX
                   "."
                   " Nmoves: %5" PRIDX ", Cut: %6" PRIDX ", Vol: %6" PRIDX,
                   pwgts[iargmin(nparts, pwgts, 1)],
                   imax(nparts, pwgts, 1),
                   ComputeLoadImbalance(graph, nparts, ctrl->pijbm),
                   graph->nbnd,
                   nmoved,
                   graph->mincut,
                   ComputeVolume(graph, where));
            if(ctrl->minconn)
                printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                       imax(nparts, nads, 1),
                       isum(nparts, nads, 1));
            printf("\n");
        }

        if(nmoved == 0 || (omode == OMODE_REFINE && graph->mincut == oldcut))
            break;
    }

    rpqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************/
/*! K-way refinement that minimizes the communication volume. This is a
    greedy routine and the vertices are visited in decreasing gv order.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves
         to violate the max-pwgt constraint.

*/
/**************************************************************************/
void Greedy_KWayVolOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode)
{
    /* Common variables to all types of kway-refinement/balancing routines */
    idx_t  i, ii, iii, j, k, l, pass, nvtxs, nparts, gain;
    idx_t  from, me, to, oldcut, vwgt;
    idx_t *xadj, *adjncy;
    idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minpwgts, *maxpwgts;
    idx_t  nmoved, nupd, *vstatus, *updptr, *updind;
    idx_t maxndoms, *safetos = NULL, *nads = NULL, *doms = NULL, **adids = NULL,
                    **adwgts = NULL;
    idx_t *bfslvl = NULL, *bfsind = NULL, *bfsmrk = NULL;
    idx_t  bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
    real_t* tpwgts;

    /* Volume-specific/different variables */
    ipq_t*     queue;
    idx_t      oldvol, xgain;
    idx_t *    vmarker, *pmarker, *modind;
    vkrinfo_t* myrinfo;
    vnbr_t*    mynbrs;

    WCOREPUSH;

    /* Link the graph fields */
    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    bndptr = graph->bndptr;
    bndind = graph->bndind;
    where  = graph->where;
    pwgts  = graph->pwgts;

    nparts = ctrl->nparts;
    tpwgts = ctrl->tpwgts;

    /* Setup the weight intervals of the various subdomains */
    minpwgts = iwspacemalloc(ctrl, nparts);
    maxpwgts = iwspacemalloc(ctrl, nparts);

    for(i = 0; i < nparts; i++)
    {
        maxpwgts[i] = ctrl->tpwgts[i] * graph->tvwgt[0] * ctrl->ubfactors[0];
        minpwgts[i] = ctrl->tpwgts[i] * graph->tvwgt[0] * (1.0 / ctrl->ubfactors[0]);
    }

    perm = iwspacemalloc(ctrl, nvtxs);


    /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made.
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
    safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

    if(ctrl->minconn)
    {
        ComputeSubDomainGraph(ctrl, graph);

        nads   = ctrl->nads;
        adids  = ctrl->adids;
        adwgts = ctrl->adwgts;
        doms   = iset(nparts, 0, ctrl->pvec1);
    }


    /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
    vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
    updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
    updind  = iwspacemalloc(ctrl, nvtxs);

    if(ctrl->contig)
    {
        /* The arrays that will be used for limited check of articulation points */
        bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
        bfsind = iwspacemalloc(ctrl, nvtxs);
        bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    }

    /* Vol-refinement specific working arrays */
    modind  = iwspacemalloc(ctrl, nvtxs);
    vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("%s: [%6" PRIDX " %6" PRIDX "]-[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL
               ", Nv-Nb[%6" PRIDX " %6" PRIDX "], Cut: %5" PRIDX ", Vol: %5" PRIDX,
               (omode == OMODE_REFINE ? "GRV" : "GBV"),
               pwgts[iargmin(nparts, pwgts, 1)],
               imax(nparts, pwgts, 1),
               minpwgts[0],
               maxpwgts[0],
               ComputeLoadImbalance(graph, nparts, ctrl->pijbm),
               graph->nvtxs,
               graph->nbnd,
               graph->mincut,
               graph->minvol);
        if(ctrl->minconn)
            printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                   imax(nparts, nads, 1),
                   isum(nparts, nads, 1));
        printf("\n");
    }

    queue = ipqCreate(nvtxs);


    /*=====================================================================
  * The top-level refinement loop
  *======================================================================*/
    for(pass = 0; pass < niter; pass++)
    {
        ASSERT(ComputeVolume(graph, where) == graph->minvol);

        if(omode == OMODE_BALANCE)
        {
            /* Check to see if things are out of balance, given the tolerance */
            for(i = 0; i < nparts; i++)
            {
                if(pwgts[i] > maxpwgts[i])
                    break;
            }
            if(i == nparts) /* Things are balanced. Return right away */
                break;
        }

        oldcut = graph->mincut;
        oldvol = graph->minvol;
        nupd   = 0;

        if(ctrl->minconn)
            maxndoms = imax(nparts, nads, 1);

        /* Insert the boundary vertices in the priority queue */
        irandArrayPermute(graph->nbnd, perm, graph->nbnd / 4, 1);
        for(ii = 0; ii < graph->nbnd; ii++)
        {
            i = bndind[perm[ii]];
            ipqInsert(queue, i, graph->vkrinfo[i].gv);
            vstatus[i] = VPQSTATUS_PRESENT;
            ListInsert(nupd, updind, updptr, i);
        }

        /* Start extracting vertices from the queue and try to move them */
        for(nmoved = 0, iii = 0;; iii++)
        {
            if((i = ipqGetTop(queue)) == -1)
                break;
            vstatus[i] = VPQSTATUS_EXTRACTED;

            myrinfo = graph->vkrinfo + i;
            mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

            from = where[i];
            vwgt = graph->vwgt[i];

            /* Prevent moves that make 'from' domain underbalanced */
            if(omode == OMODE_REFINE)
            {
                if(myrinfo->nid > 0 && pwgts[from] - vwgt < minpwgts[from])
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                if(pwgts[from] - vwgt < minpwgts[from])
                    continue;
            }

            if(ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                continue;

            if(ctrl->minconn)
                SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

            xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? graph->vsize[i] : 0);

            /* Find the most promising subdomain to move to */
            if(omode == OMODE_REFINE)
            {
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    gain = mynbrs[k].gv + xgain;
                    if(gain >= 0 && pwgts[to] + vwgt <= maxpwgts[to] + ffactor * gain)
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    gain = mynbrs[j].gv + xgain;
                    if((mynbrs[j].gv > mynbrs[k].gv
                        && pwgts[to] + vwgt <= maxpwgts[to] + ffactor * gain)
                       || (mynbrs[j].gv == mynbrs[k].gv
                           && mynbrs[j].ned > mynbrs[k].ned
                           && pwgts[to] + vwgt <= maxpwgts[to])
                       || (mynbrs[j].gv == mynbrs[k].gv
                           && mynbrs[j].ned == mynbrs[k].ned
                           && tpwgts[mynbrs[k].pid] * pwgts[to]
                                  < tpwgts[to] * pwgts[mynbrs[k].pid]))
                        k = j;
                }
                to = mynbrs[k].pid;

                ASSERT(xgain + mynbrs[k].gv >= 0);

                j = 0;
                if(xgain + mynbrs[k].gv > 0 || mynbrs[k].ned - myrinfo->nid > 0)
                    j = 1;
                else if(mynbrs[k].ned - myrinfo->nid == 0)
                {
                    if((iii % 2 == 0 && safetos[to] == 2) || pwgts[from] >= maxpwgts[from]
                       || tpwgts[from] * (pwgts[to] + vwgt) < tpwgts[to] * pwgts[from])
                        j = 1;
                }
                if(j == 0)
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    if(pwgts[to] + vwgt <= maxpwgts[to]
                       || tpwgts[from] * (pwgts[to] + vwgt) <= tpwgts[to] * pwgts[from])
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    if(tpwgts[mynbrs[k].pid] * pwgts[to]
                       < tpwgts[to] * pwgts[mynbrs[k].pid])
                        k = j;
                }
                to = mynbrs[k].pid;

                if(pwgts[from] < maxpwgts[from] && pwgts[to] > minpwgts[to]
                   && (xgain + mynbrs[k].gv < 0
                       || (xgain + mynbrs[k].gv == 0 && mynbrs[k].ned - myrinfo->nid < 0)))
                    continue;
            }


            /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to'
      *======================================================================*/
            INC_DEC(pwgts[to], pwgts[from], vwgt);
            graph->mincut -= mynbrs[k].ned - myrinfo->nid;
            graph->minvol -= (xgain + mynbrs[k].gv);
            where[i] = to;
            nmoved++;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("\t\tMoving %6" PRIDX " from %3" PRIDX " to %3" PRIDX ". "
                         "Gain: [%4" PRIDX " %4" PRIDX "]. Cut: %6" PRIDX ", Vol: %6" PRIDX "\n",
                         i,
                         from,
                         to,
                         xgain + mynbrs[k].gv,
                         mynbrs[k].ned - myrinfo->nid,
                         graph->mincut,
                         graph->minvol));

            /* Update the subdomain connectivity information */
            if(ctrl->minconn)
            {
                /* take care of i's move itself */
                UpdateEdgeSubDomainGraph(
                    ctrl, from, to, myrinfo->nid - mynbrs[k].ned, &maxndoms);

                /* take care of the adjacent vertices */
                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    me = where[adjncy[j]];
                    if(me != from && me != to)
                    {
                        UpdateEdgeSubDomainGraph(ctrl, from, me, -1, &maxndoms);
                        UpdateEdgeSubDomainGraph(ctrl, to, me, 1, &maxndoms);
                    }
                }
            }

            /* Update the id/ed/gains/bnd/queue of potentially affected nodes */
            KWayVolUpdate(
                ctrl, graph, i, from, to, queue, vstatus, &nupd, updptr, updind, bndtype, vmarker, pmarker, modind);

            /*CheckKWayVolPartitionParams(ctrl, graph); */
        }


        /* Reset the vstatus and associated data structures */
        for(i = 0; i < nupd; i++)
        {
            ASSERT(updptr[updind[i]] != -1);
            ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
            vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
            updptr[updind[i]]  = -1;
        }

        if(ctrl->dbglvl & METIS_DBG_REFINE)
        {
            printf("\t[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL ", Nb: %6" PRIDX
                   "."
                   " Nmoves: %5" PRIDX ", Cut: %6" PRIDX ", Vol: %6" PRIDX,
                   pwgts[iargmin(nparts, pwgts, 1)],
                   imax(nparts, pwgts, 1),
                   ComputeLoadImbalance(graph, nparts, ctrl->pijbm),
                   graph->nbnd,
                   nmoved,
                   graph->mincut,
                   graph->minvol);
            if(ctrl->minconn)
                printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                       imax(nparts, nads, 1),
                       isum(nparts, nads, 1));
            printf("\n");
        }

        if(nmoved == 0
           || (omode == OMODE_REFINE && graph->minvol == oldvol && graph->mincut == oldcut))
            break;
    }

    ipqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************/
/*! K-way partitioning optimization in which the vertices are visited in
    decreasing ed/sqrt(nnbrs)-id order. Note this is just an
    approximation, as the ed is often split across different subdomains
    and the sqrt(nnbrs) is just a crude approximation.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves
         to violate the max-pwgt constraint.
  \param omode is the type of optimization that will performed among
         OMODE_REFINE and OMODE_BALANCE


*/
/**************************************************************************/
void Greedy_McKWayCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode)
{
    /* Common variables to all types of kway-refinement/balancing routines */
    idx_t  i, ii, iii, j, k, l, pass, nvtxs, ncon, nparts, gain;
    idx_t  from, me, to, cto, oldcut;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt;
    idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minpwgts, *maxpwgts;
    idx_t  nmoved, nupd, *vstatus, *updptr, *updind;
    idx_t maxndoms, *safetos = NULL, *nads = NULL, *doms = NULL, **adids = NULL,
                    **adwgts = NULL;
    idx_t *bfslvl = NULL, *bfsind = NULL, *bfsmrk = NULL;
    idx_t  bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
    real_t *ubfactors, *pijbm;
    real_t  origbal;

    /* Edgecut-specific/different variables */
    idx_t      nbnd, oldnnbrs;
    rpq_t*     queue;
    real_t     rgain;
    ckrinfo_t* myrinfo;
    cnbr_t*    mynbrs;

    WCOREPUSH;

    /* Link the graph fields */
    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;

    where = graph->where;
    pwgts = graph->pwgts;

    nparts = ctrl->nparts;
    pijbm  = ctrl->pijbm;


    /* Determine the ubfactors. The method used is different based on omode.
     When OMODE_BALANCE, the ubfactors are those supplied by the user.
     When OMODE_REFINE, the ubfactors are the max of the current partition
     and the user-specified ones. */
    ubfactors = rwspacemalloc(ctrl, ncon);
    ComputeLoadImbalanceVec(graph, nparts, pijbm, ubfactors);
    origbal = rvecmaxdiff(ncon, ubfactors, ctrl->ubfactors);
    if(omode == OMODE_BALANCE)
    {
        rcopy(ncon, ctrl->ubfactors, ubfactors);
    }
    else
    {
        for(i = 0; i < ncon; i++)
            ubfactors[i] = (ubfactors[i] > ctrl->ubfactors[i] ? ubfactors[i] :
                                                                ctrl->ubfactors[i]);
    }


    /* Setup the weight intervals of the various subdomains */
    minpwgts = iwspacemalloc(ctrl, nparts * ncon);
    maxpwgts = iwspacemalloc(ctrl, nparts * ncon);

    for(i = 0; i < nparts; i++)
    {
        for(j = 0; j < ncon; j++)
        {
            maxpwgts[i * ncon + j] =
                ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * ubfactors[j];
            /*minpwgts[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*(.9/ubfactors[j]);*/
            minpwgts[i * ncon + j] = ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * .2;
        }
    }

    perm = iwspacemalloc(ctrl, nvtxs);


    /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made.
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
    safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

    if(ctrl->minconn)
    {
        ComputeSubDomainGraph(ctrl, graph);

        nads   = ctrl->nads;
        adids  = ctrl->adids;
        adwgts = ctrl->adwgts;
        doms   = iset(nparts, 0, ctrl->pvec1);
    }


    /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
    vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
    updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
    updind  = iwspacemalloc(ctrl, nvtxs);

    if(ctrl->contig)
    {
        /* The arrays that will be used for limited check of articulation points */
        bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
        bfsind = iwspacemalloc(ctrl, nvtxs);
        bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    }

    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("%s: [%6" PRIDX " %6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL "(%.3" PRREAL
               "),"
               " Nv-Nb[%6" PRIDX " %6" PRIDX "], Cut: %6" PRIDX ", (%" PRIDX ")",
               (omode == OMODE_REFINE ? "GRC" : "GBC"),
               imin(nparts * ncon, pwgts, 1),
               imax(nparts * ncon, pwgts, 1),
               imax(nparts * ncon, maxpwgts, 1),
               ComputeLoadImbalance(graph, nparts, pijbm),
               origbal,
               graph->nvtxs,
               graph->nbnd,
               graph->mincut,
               niter);
        if(ctrl->minconn)
            printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                   imax(nparts, nads, 1),
                   isum(nparts, nads, 1));
        printf("\n");
    }

    queue = rpqCreate(nvtxs);


    /*=====================================================================
  * The top-level refinement loop
  *======================================================================*/
    for(pass = 0; pass < niter; pass++)
    {
        ASSERT(ComputeCut(graph, where) == graph->mincut);
        if(omode == OMODE_REFINE)
            ASSERT(CheckBnd2(graph));

        /* In balancing mode, exit as soon as balance is reached */
        if(omode == OMODE_BALANCE && IsBalanced(ctrl, graph, 0))
            break;

        oldcut = graph->mincut;
        nbnd   = graph->nbnd;
        nupd   = 0;

        if(ctrl->minconn)
            maxndoms = imax(nparts, nads, 1);

        /* Insert the boundary vertices in the priority queue */
        irandArrayPermute(nbnd, perm, nbnd / 4, 1);
        for(ii = 0; ii < nbnd; ii++)
        {
            i     = bndind[perm[ii]];
            rgain = (graph->ckrinfo[i].nnbrs > 0 ?
                         1.0 * graph->ckrinfo[i].ed / sqrt(graph->ckrinfo[i].nnbrs) :
                         0.0)
                    - graph->ckrinfo[i].id;
            rpqInsert(queue, i, rgain);
            vstatus[i] = VPQSTATUS_PRESENT;
            ListInsert(nupd, updind, updptr, i);
        }

        /* Start extracting vertices from the queue and try to move them */
        for(nmoved = 0, iii = 0;; iii++)
        {
            if((i = rpqGetTop(queue)) == -1)
                break;
            vstatus[i] = VPQSTATUS_EXTRACTED;

            myrinfo = graph->ckrinfo + i;
            mynbrs  = ctrl->cnbrpool + myrinfo->inbr;

            from = where[i];

            /* Prevent moves that make 'from' domain underbalanced */
            if(omode == OMODE_REFINE)
            {
                if(myrinfo->id > 0
                   && !ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts + from * ncon))
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                if(!ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts + from * ncon))
                    continue;
            }

            if(ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                continue;

            if(ctrl->minconn)
                SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

            /* Find the most promising subdomain to move to */
            if(omode == OMODE_REFINE)
            {
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    gain = mynbrs[k].ed - myrinfo->id;
                    if(gain >= 0
                       && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon))
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                cto = to;
                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    if((mynbrs[j].ed > mynbrs[k].ed
                        && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon))
                       || (mynbrs[j].ed == mynbrs[k].ed
                           && BetterBalanceKWay(ncon,
                                                vwgt + i * ncon,
                                                ubfactors,
                                                1,
                                                pwgts + cto * ncon,
                                                pijbm + cto * ncon,
                                                1,
                                                pwgts + to * ncon,
                                                pijbm + to * ncon)))
                    {
                        k   = j;
                        cto = to;
                    }
                }
                to = cto;

                gain = mynbrs[k].ed - myrinfo->id;
                if(!(gain > 0
                     || (gain == 0
                         && (BetterBalanceKWay(ncon,
                                               vwgt + i * ncon,
                                               ubfactors,
                                               -1,
                                               pwgts + from * ncon,
                                               pijbm + from * ncon,
                                               +1,
                                               pwgts + to * ncon,
                                               pijbm + to * ncon)
                             || (iii % 2 == 0 && safetos[to] == 2)))))
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    if(ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon)
                       || BetterBalanceKWay(ncon,
                                            vwgt + i * ncon,
                                            ubfactors,
                                            -1,
                                            pwgts + from * ncon,
                                            pijbm + from * ncon,
                                            +1,
                                            pwgts + to * ncon,
                                            pijbm + to * ncon))
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                cto = to;
                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    if(BetterBalanceKWay(ncon,
                                         vwgt + i * ncon,
                                         ubfactors,
                                         1,
                                         pwgts + cto * ncon,
                                         pijbm + cto * ncon,
                                         1,
                                         pwgts + to * ncon,
                                         pijbm + to * ncon))
                    {
                        k   = j;
                        cto = to;
                    }
                }
                to = cto;

                if(mynbrs[k].ed - myrinfo->id < 0
                   && !BetterBalanceKWay(ncon,
                                         vwgt + i * ncon,
                                         ubfactors,
                                         -1,
                                         pwgts + from * ncon,
                                         pijbm + from * ncon,
                                         +1,
                                         pwgts + to * ncon,
                                         pijbm + to * ncon))
                    continue;
            }


            /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to'
      *======================================================================*/
            graph->mincut -= mynbrs[k].ed - myrinfo->id;
            nmoved++;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("\t\tMoving %6" PRIDX " to %3" PRIDX ". Gain: %4" PRIDX
                         ". Cut: %6" PRIDX "\n",
                         i,
                         to,
                         mynbrs[k].ed - myrinfo->id,
                         graph->mincut));

            /* Update the subdomain connectivity information */
            if(ctrl->minconn)
            {
                /* take care of i's move itself */
                UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id - mynbrs[k].ed, &maxndoms);

                /* take care of the adjacent vertices */
                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    me = where[adjncy[j]];
                    if(me != from && me != to)
                    {
                        UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], &maxndoms);
                        UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], &maxndoms);
                    }
                }
            }

            /* Update ID/ED and BND related information for the moved vertex */
            iaxpy(ncon, 1, vwgt + i * ncon, 1, pwgts + to * ncon, 1);
            iaxpy(ncon, -1, vwgt + i * ncon, 1, pwgts + from * ncon, 1);
            UpdateMovedVertexInfoAndBND(
                i, from, k, to, myrinfo, mynbrs, where, nbnd, bndptr, bndind, bndtype);

            /* Update the degrees of adjacent vertices */
            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                ii      = adjncy[j];
                me      = where[ii];
                myrinfo = graph->ckrinfo + ii;

                oldnnbrs = myrinfo->nnbrs;

                UpdateAdjacentVertexInfoAndBND(
                    ctrl, ii, xadj[ii + 1] - xadj[ii], me, from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

                UpdateQueueInfo(
                    queue, vstatus, ii, me, from, to, myrinfo, oldnnbrs, nupd, updptr, updind, bndtype);

                ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
            }
        }

        graph->nbnd = nbnd;

        /* Reset the vstatus and associated data structures */
        for(i = 0; i < nupd; i++)
        {
            ASSERT(updptr[updind[i]] != -1);
            ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
            vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
            updptr[updind[i]]  = -1;
        }

        if(ctrl->dbglvl & METIS_DBG_REFINE)
        {
            printf("\t[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL ", Nb: %6" PRIDX
                   "."
                   " Nmoves: %5" PRIDX ", Cut: %6" PRIDX ", Vol: %6" PRIDX,
                   imin(nparts * ncon, pwgts, 1),
                   imax(nparts * ncon, pwgts, 1),
                   ComputeLoadImbalance(graph, nparts, pijbm),
                   graph->nbnd,
                   nmoved,
                   graph->mincut,
                   ComputeVolume(graph, where));
            if(ctrl->minconn)
                printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                       imax(nparts, nads, 1),
                       isum(nparts, nads, 1));
            printf("\n");
        }

        if(nmoved == 0 || (omode == OMODE_REFINE && graph->mincut == oldcut))
            break;
    }

    rpqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************/
/*! K-way refinement that minimizes the communication volume. This is a
    greedy routine and the vertices are visited in decreasing gv order.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves
         to violate the max-pwgt constraint.

*/
/**************************************************************************/
void Greedy_McKWayVolOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter, real_t ffactor, idx_t omode)
{
    /* Common variables to all types of kway-refinement/balancing routines */
    idx_t  i, ii, iii, j, k, l, pass, nvtxs, ncon, nparts, gain;
    idx_t  from, me, to, cto, oldcut;
    idx_t *xadj, *vwgt, *adjncy;
    idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minpwgts, *maxpwgts;
    idx_t  nmoved, nupd, *vstatus, *updptr, *updind;
    idx_t maxndoms, *safetos = NULL, *nads = NULL, *doms = NULL, **adids = NULL,
                    **adwgts = NULL;
    idx_t *bfslvl = NULL, *bfsind = NULL, *bfsmrk = NULL;
    idx_t  bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
    real_t *ubfactors, *pijbm;
    real_t  origbal;

    /* Volume-specific/different variables */
    ipq_t*     queue;
    idx_t      oldvol, xgain;
    idx_t *    vmarker, *pmarker, *modind;
    vkrinfo_t* myrinfo;
    vnbr_t*    mynbrs;

    WCOREPUSH;

    /* Link the graph fields */
    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    bndptr = graph->bndptr;
    bndind = graph->bndind;
    where  = graph->where;
    pwgts  = graph->pwgts;

    nparts = ctrl->nparts;
    pijbm  = ctrl->pijbm;


    /* Determine the ubfactors. The method used is different based on omode.
     When OMODE_BALANCE, the ubfactors are those supplied by the user.
     When OMODE_REFINE, the ubfactors are the max of the current partition
     and the user-specified ones. */
    ubfactors = rwspacemalloc(ctrl, ncon);
    ComputeLoadImbalanceVec(graph, nparts, pijbm, ubfactors);
    origbal = rvecmaxdiff(ncon, ubfactors, ctrl->ubfactors);
    if(omode == OMODE_BALANCE)
    {
        rcopy(ncon, ctrl->ubfactors, ubfactors);
    }
    else
    {
        for(i = 0; i < ncon; i++)
            ubfactors[i] = (ubfactors[i] > ctrl->ubfactors[i] ? ubfactors[i] :
                                                                ctrl->ubfactors[i]);
    }


    /* Setup the weight intervals of the various subdomains */
    minpwgts = iwspacemalloc(ctrl, nparts * ncon);
    maxpwgts = iwspacemalloc(ctrl, nparts * ncon);

    for(i = 0; i < nparts; i++)
    {
        for(j = 0; j < ncon; j++)
        {
            maxpwgts[i * ncon + j] =
                ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * ubfactors[j];
            /*minpwgts[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*(.9/ubfactors[j]); */
            minpwgts[i * ncon + j] = ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * .2;
        }
    }

    perm = iwspacemalloc(ctrl, nvtxs);


    /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made.
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
    safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

    if(ctrl->minconn)
    {
        ComputeSubDomainGraph(ctrl, graph);

        nads   = ctrl->nads;
        adids  = ctrl->adids;
        adwgts = ctrl->adwgts;
        doms   = iset(nparts, 0, ctrl->pvec1);
    }


    /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
    vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
    updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
    updind  = iwspacemalloc(ctrl, nvtxs);

    if(ctrl->contig)
    {
        /* The arrays that will be used for limited check of articulation points */
        bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
        bfsind = iwspacemalloc(ctrl, nvtxs);
        bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    }

    /* Vol-refinement specific working arrays */
    modind  = iwspacemalloc(ctrl, nvtxs);
    vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("%s: [%6" PRIDX " %6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL "(%.3" PRREAL
               "),"
               ", Nv-Nb[%6" PRIDX " %6" PRIDX "], Cut: %5" PRIDX
               ", Vol: %5" PRIDX ", (%" PRIDX ")",
               (omode == OMODE_REFINE ? "GRV" : "GBV"),
               imin(nparts * ncon, pwgts, 1),
               imax(nparts * ncon, pwgts, 1),
               imax(nparts * ncon, maxpwgts, 1),
               ComputeLoadImbalance(graph, nparts, pijbm),
               origbal,
               graph->nvtxs,
               graph->nbnd,
               graph->mincut,
               graph->minvol,
               niter);
        if(ctrl->minconn)
            printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                   imax(nparts, nads, 1),
                   isum(nparts, nads, 1));
        printf("\n");
    }

    queue = ipqCreate(nvtxs);


    /*=====================================================================
  * The top-level refinement loop
  *======================================================================*/
    for(pass = 0; pass < niter; pass++)
    {
        ASSERT(ComputeVolume(graph, where) == graph->minvol);

        /* In balancing mode, exit as soon as balance is reached */
        if(omode == OMODE_BALANCE && IsBalanced(ctrl, graph, 0))
            break;

        oldcut = graph->mincut;
        oldvol = graph->minvol;
        nupd   = 0;

        if(ctrl->minconn)
            maxndoms = imax(nparts, nads, 1);

        /* Insert the boundary vertices in the priority queue */
        irandArrayPermute(graph->nbnd, perm, graph->nbnd / 4, 1);
        for(ii = 0; ii < graph->nbnd; ii++)
        {
            i = bndind[perm[ii]];
            ipqInsert(queue, i, graph->vkrinfo[i].gv);
            vstatus[i] = VPQSTATUS_PRESENT;
            ListInsert(nupd, updind, updptr, i);
        }

        /* Start extracting vertices from the queue and try to move them */
        for(nmoved = 0, iii = 0;; iii++)
        {
            if((i = ipqGetTop(queue)) == -1)
                break;
            vstatus[i] = VPQSTATUS_EXTRACTED;

            myrinfo = graph->vkrinfo + i;
            mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

            from = where[i];

            /* Prevent moves that make 'from' domain underbalanced */
            if(omode == OMODE_REFINE)
            {
                if(myrinfo->nid > 0
                   && !ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts + from * ncon))
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                if(!ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts + from * ncon))
                    continue;
            }

            if(ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                continue;

            if(ctrl->minconn)
                SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

            xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? graph->vsize[i] : 0);

            /* Find the most promising subdomain to move to */
            if(omode == OMODE_REFINE)
            {
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    gain = mynbrs[k].gv + xgain;
                    if(gain >= 0
                       && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon))
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                cto = to;
                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    gain = mynbrs[j].gv + xgain;
                    if((mynbrs[j].gv > mynbrs[k].gv
                        && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon))
                       || (mynbrs[j].gv == mynbrs[k].gv
                           && mynbrs[j].ned > mynbrs[k].ned
                           && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon))
                       || (mynbrs[j].gv == mynbrs[k].gv
                           && mynbrs[j].ned == mynbrs[k].ned
                           && BetterBalanceKWay(ncon,
                                                vwgt + i * ncon,
                                                ubfactors,
                                                1,
                                                pwgts + cto * ncon,
                                                pijbm + cto * ncon,
                                                1,
                                                pwgts + to * ncon,
                                                pijbm + to * ncon)))
                    {
                        k   = j;
                        cto = to;
                    }
                }
                to = cto;

                j = 0;
                if(xgain + mynbrs[k].gv > 0 || mynbrs[k].ned - myrinfo->nid > 0)
                    j = 1;
                else if(mynbrs[k].ned - myrinfo->nid == 0)
                {
                    if((iii % 2 == 0 && safetos[to] == 2)
                       || BetterBalanceKWay(ncon,
                                            vwgt + i * ncon,
                                            ubfactors,
                                            -1,
                                            pwgts + from * ncon,
                                            pijbm + from * ncon,
                                            +1,
                                            pwgts + to * ncon,
                                            pijbm + to * ncon))
                        j = 1;
                }
                if(j == 0)
                    continue;
            }
            else
            { /* OMODE_BALANCE */
                for(k = myrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(!safetos[to = mynbrs[k].pid])
                        continue;
                    if(ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts + to * ncon)
                       || BetterBalanceKWay(ncon,
                                            vwgt + i * ncon,
                                            ubfactors,
                                            -1,
                                            pwgts + from * ncon,
                                            pijbm + from * ncon,
                                            +1,
                                            pwgts + to * ncon,
                                            pijbm + to * ncon))
                        break;
                }
                if(k < 0)
                    continue; /* break out if you did not find a candidate */

                cto = to;
                for(j = k - 1; j >= 0; j--)
                {
                    if(!safetos[to = mynbrs[j].pid])
                        continue;
                    if(BetterBalanceKWay(ncon,
                                         vwgt + i * ncon,
                                         ubfactors,
                                         1,
                                         pwgts + cto * ncon,
                                         pijbm + cto * ncon,
                                         1,
                                         pwgts + to * ncon,
                                         pijbm + to * ncon))
                    {
                        k   = j;
                        cto = to;
                    }
                }
                to = cto;

                if((xgain + mynbrs[k].gv < 0
                    || (xgain + mynbrs[k].gv == 0 && mynbrs[k].ned - myrinfo->nid < 0))
                   && !BetterBalanceKWay(ncon,
                                         vwgt + i * ncon,
                                         ubfactors,
                                         -1,
                                         pwgts + from * ncon,
                                         pijbm + from * ncon,
                                         +1,
                                         pwgts + to * ncon,
                                         pijbm + to * ncon))
                    continue;
            }


            /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to'
      *======================================================================*/
            graph->mincut -= mynbrs[k].ned - myrinfo->nid;
            graph->minvol -= (xgain + mynbrs[k].gv);
            where[i] = to;
            nmoved++;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("\t\tMoving %6" PRIDX " from %3" PRIDX " to %3" PRIDX ". "
                         "Gain: [%4" PRIDX " %4" PRIDX "]. Cut: %6" PRIDX ", Vol: %6" PRIDX "\n",
                         i,
                         from,
                         to,
                         xgain + mynbrs[k].gv,
                         mynbrs[k].ned - myrinfo->nid,
                         graph->mincut,
                         graph->minvol));

            /* Update the subdomain connectivity information */
            if(ctrl->minconn)
            {
                /* take care of i's move itself */
                UpdateEdgeSubDomainGraph(
                    ctrl, from, to, myrinfo->nid - mynbrs[k].ned, &maxndoms);

                /* take care of the adjacent vertices */
                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    me = where[adjncy[j]];
                    if(me != from && me != to)
                    {
                        UpdateEdgeSubDomainGraph(ctrl, from, me, -1, &maxndoms);
                        UpdateEdgeSubDomainGraph(ctrl, to, me, 1, &maxndoms);
                    }
                }
            }

            /* Update pwgts */
            iaxpy(ncon, 1, vwgt + i * ncon, 1, pwgts + to * ncon, 1);
            iaxpy(ncon, -1, vwgt + i * ncon, 1, pwgts + from * ncon, 1);

            /* Update the id/ed/gains/bnd/queue of potentially affected nodes */
            KWayVolUpdate(
                ctrl, graph, i, from, to, queue, vstatus, &nupd, updptr, updind, bndtype, vmarker, pmarker, modind);

            /*CheckKWayVolPartitionParams(ctrl, graph); */
        }


        /* Reset the vstatus and associated data structures */
        for(i = 0; i < nupd; i++)
        {
            ASSERT(updptr[updind[i]] != -1);
            ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
            vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
            updptr[updind[i]]  = -1;
        }

        if(ctrl->dbglvl & METIS_DBG_REFINE)
        {
            printf("\t[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL ", Nb: %6" PRIDX
                   "."
                   " Nmoves: %5" PRIDX ", Cut: %6" PRIDX ", Vol: %6" PRIDX,
                   imin(nparts * ncon, pwgts, 1),
                   imax(nparts * ncon, pwgts, 1),
                   ComputeLoadImbalance(graph, nparts, pijbm),
                   graph->nbnd,
                   nmoved,
                   graph->mincut,
                   graph->minvol);
            if(ctrl->minconn)
                printf(", Doms: [%3" PRIDX " %4" PRIDX "]",
                       imax(nparts, nads, 1),
                       isum(nparts, nads, 1));
            printf("\n");
        }

        if(nmoved == 0
           || (omode == OMODE_REFINE && graph->minvol == oldvol && graph->mincut == oldcut))
            break;
    }

    ipqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************/
/*! This function performs an approximate articulation vertex test.
    It assumes that the bfslvl, bfsind, and bfsmrk arrays are initialized
    appropriately. */
/*************************************************************************/
idx_t IsArticulationNode(
    idx_t i, idx_t* xadj, idx_t* adjncy, idx_t* where, idx_t* bfslvl, idx_t* bfsind, idx_t* bfsmrk)
{
    idx_t ii, j, k = 0, head, tail, nhits, tnhits, from, BFSDEPTH = 5;

    from = where[i];

    /* Determine if the vertex is safe to move from a contiguity standpoint */
    for(tnhits = 0, j = xadj[i]; j < xadj[i + 1]; j++)
    {
        if(where[adjncy[j]] == from)
        {
            ASSERT(bfsmrk[adjncy[j]] == 0);
            ASSERT(bfslvl[adjncy[j]] == 0);
            bfsmrk[k = adjncy[j]] = 1;
            tnhits++;
        }
    }

    /* Easy cases */
    if(tnhits == 0)
        return 0;
    if(tnhits == 1)
    {
        bfsmrk[k] = 0;
        return 0;
    }

    ASSERT(bfslvl[i] == 0);
    bfslvl[i] = 1;

    bfsind[0] = k; /* That was the last one from the previous loop */
    bfslvl[k] = 1;
    bfsmrk[k] = 0;
    head      = 0;
    tail      = 1;

    /* Do a limited BFS traversal to see if you can get to all the other nodes */
    for(nhits = 1; head < tail;)
    {
        ii = bfsind[head++];
        for(j = xadj[ii]; j < xadj[ii + 1]; j++)
        {
            if(where[k = adjncy[j]] == from)
            {
                if(bfsmrk[k])
                {
                    bfsmrk[k] = 0;
                    if(++nhits == tnhits)
                        break;
                }
                if(bfslvl[k] == 0 && bfslvl[ii] < BFSDEPTH)
                {
                    bfsind[tail++] = k;
                    bfslvl[k]      = bfslvl[ii] + 1;
                }
            }
        }
        if(nhits == tnhits)
            break;
    }

    /* Reset the various BFS related arrays */
    bfslvl[i] = 0;
    for(j = 0; j < tail; j++)
        bfslvl[bfsind[j]] = 0;


    /* Reset the bfsmrk array for the next vertex when has not already being cleared */
    if(nhits < tnhits)
    {
        for(j = xadj[i]; j < xadj[i + 1]; j++)
            if(where[adjncy[j]] == from)
                bfsmrk[adjncy[j]] = 0;
    }

    return (nhits != tnhits);
}


/*************************************************************************/
/*!
 This function updates the edge and volume gains due to a vertex movement.
 v from 'from' to 'to'.

 \param ctrl is the control structure.
 \param graph is the graph being partitioned.
 \param v is the vertex that is moving.
 \param from is the original partition of v.
 \param to is the new partition of v.
 \param queue is the priority queue. If the queue is NULL, no priority-queue
        related updates are performed.
 \param vstatus is an array that marks the status of the vertex in terms
        of the priority queue. If queue is NULL, this parameter is ignored.
 \param r_nqupd is the number of vertices that have been inserted/removed
        from the queue. If queue is NULL, this parameter is ignored.
 \param updptr stores the index of each vertex in updind. If queue is NULL,
        this parameter is ignored.
 \param updind is the list of vertices that have been inserted/removed from
        the queue. If queue is NULL, this parameter is ignored.
 \param vmarker is of size nvtxs and is used internally as a temporary array.
        On entry and return all of its entries are 0.
 \param pmarker is of size nparts and is used internally as a temporary marking
        array. On entry and return all of its entries are -1.
 \param modind is an array of size nvtxs and is used to keep track of the
        list of vertices whose gains need to be updated.
*/
/*************************************************************************/
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
                   idx_t*   modind)
{
    idx_t      i, ii, iii, j, jj, k, kk, l, u, nmod, other, me, myidx;
    idx_t *    xadj, *vsize, *adjncy, *where;
    vkrinfo_t *myrinfo, *orinfo;
    vnbr_t *   mynbrs, *onbrs;

    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vsize  = graph->vsize;
    where  = graph->where;

    myrinfo = graph->vkrinfo + v;
    mynbrs  = ctrl->vnbrpool + myrinfo->inbr;


    /*======================================================================
   * Remove the contributions on the gain made by 'v'.
   *=====================================================================*/
    for(k = 0; k < myrinfo->nnbrs; k++)
        pmarker[mynbrs[k].pid] = k;
    pmarker[from] = k;

    myidx = pmarker[to]; /* Keep track of the index in mynbrs of the 'to' domain */

    for(j = xadj[v]; j < xadj[v + 1]; j++)
    {
        ii     = adjncy[j];
        other  = where[ii];
        orinfo = graph->vkrinfo + ii;
        onbrs  = ctrl->vnbrpool + orinfo->inbr;

        if(other == from)
        {
            for(k = 0; k < orinfo->nnbrs; k++)
            {
                if(pmarker[onbrs[k].pid] == -1)
                    onbrs[k].gv += vsize[v];
            }
        }
        else
        {
            ASSERT(pmarker[other] != -1);

            if(mynbrs[pmarker[other]].ned > 1)
            {
                for(k = 0; k < orinfo->nnbrs; k++)
                {
                    if(pmarker[onbrs[k].pid] == -1)
                        onbrs[k].gv += vsize[v];
                }
            }
            else
            { /* There is only one connection */
                for(k = 0; k < orinfo->nnbrs; k++)
                {
                    if(pmarker[onbrs[k].pid] != -1)
                        onbrs[k].gv -= vsize[v];
                }
            }
        }
    }

    for(k = 0; k < myrinfo->nnbrs; k++)
        pmarker[mynbrs[k].pid] = -1;
    pmarker[from] = -1;


    /*======================================================================
   * Update the id/ed of vertex 'v'
   *=====================================================================*/
    if(myidx == -1)
    {
        myidx = myrinfo->nnbrs++;
        ASSERT(myidx < xadj[v + 1] - xadj[v]);
        mynbrs[myidx].ned = 0;
    }
    myrinfo->ned += myrinfo->nid - mynbrs[myidx].ned;
    SWAP(myrinfo->nid, mynbrs[myidx].ned, j);
    if(mynbrs[myidx].ned == 0)
        mynbrs[myidx] = mynbrs[--myrinfo->nnbrs];
    else
        mynbrs[myidx].pid = from;


    /*======================================================================
   * Update the degrees of adjacent vertices and their volume gains
   *=====================================================================*/
    vmarker[v] = 1;
    modind[0]  = v;
    nmod       = 1;
    for(j = xadj[v]; j < xadj[v + 1]; j++)
    {
        ii = adjncy[j];
        me = where[ii];

        if(!vmarker[ii])
        { /* The marking is done for boundary and max gv calculations */
            vmarker[ii]    = 2;
            modind[nmod++] = ii;
        }

        myrinfo = graph->vkrinfo + ii;
        if(myrinfo->inbr == -1)
            myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[ii + 1] - xadj[ii]);
        mynbrs = ctrl->vnbrpool + myrinfo->inbr;

        if(me == from)
        {
            INC_DEC(myrinfo->ned, myrinfo->nid, 1);
        }
        else if(me == to)
        {
            INC_DEC(myrinfo->nid, myrinfo->ned, 1);
        }

        /* Remove the edgeweight from the 'pid == from' entry of the vertex */
        if(me != from)
        {
            for(k = 0; k < myrinfo->nnbrs; k++)
            {
                if(mynbrs[k].pid == from)
                {
                    if(mynbrs[k].ned == 1)
                    {
                        mynbrs[k]   = mynbrs[--myrinfo->nnbrs];
                        vmarker[ii] = 1; /* You do a complete .gv calculation */

                        /* All vertices adjacent to 'ii' need to be updated */
                        for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                        {
                            u      = adjncy[jj];
                            other  = where[u];
                            orinfo = graph->vkrinfo + u;
                            onbrs  = ctrl->vnbrpool + orinfo->inbr;

                            for(kk = 0; kk < orinfo->nnbrs; kk++)
                            {
                                if(onbrs[kk].pid == from)
                                {
                                    onbrs[kk].gv -= vsize[ii];
                                    if(!vmarker[u])
                                    { /* Need to update boundary etc */
                                        vmarker[u]     = 2;
                                        modind[nmod++] = u;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        mynbrs[k].ned--;

                        /* Update the gv due to single 'ii' connection to 'from' */
                        if(mynbrs[k].ned == 1)
                        {
                            /* find the vertex 'u' that 'ii' was connected into 'from' */
                            for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                            {
                                u     = adjncy[jj];
                                other = where[u];

                                if(other == from)
                                {
                                    orinfo = graph->vkrinfo + u;
                                    onbrs  = ctrl->vnbrpool + orinfo->inbr;

                                    /* The following is correct because domains in common
                     between ii and u will lead to a reduction over the
                     previous gain, whereas domains only in u but not in
                     ii, will lead to no change as opposed to the earlier
                     increase */
                                    for(kk = 0; kk < orinfo->nnbrs; kk++)
                                        onbrs[kk].gv += vsize[ii];

                                    if(!vmarker[u])
                                    { /* Need to update boundary etc */
                                        vmarker[u]     = 2;
                                        modind[nmod++] = u;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
            }
        }


        /* Add the edgeweight to the 'pid == to' entry of the vertex */
        if(me != to)
        {
            for(k = 0; k < myrinfo->nnbrs; k++)
            {
                if(mynbrs[k].pid == to)
                {
                    mynbrs[k].ned++;

                    /* Update the gv due to non-single 'ii' connection to 'to' */
                    if(mynbrs[k].ned == 2)
                    {
                        /* find the vertex 'u' that 'ii' was connected into 'to' */
                        for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                        {
                            u     = adjncy[jj];
                            other = where[u];

                            if(u != v && other == to)
                            {
                                orinfo = graph->vkrinfo + u;
                                onbrs  = ctrl->vnbrpool + orinfo->inbr;
                                for(kk = 0; kk < orinfo->nnbrs; kk++)
                                    onbrs[kk].gv -= vsize[ii];

                                if(!vmarker[u])
                                { /* Need to update boundary etc */
                                    vmarker[u]     = 2;
                                    modind[nmod++] = u;
                                }
                                break;
                            }
                        }
                    }
                    break;
                }
            }

            if(k == myrinfo->nnbrs)
            {
                mynbrs[myrinfo->nnbrs].pid   = to;
                mynbrs[myrinfo->nnbrs++].ned = 1;
                vmarker[ii] = 1; /* You do a complete .gv calculation */

                /* All vertices adjacent to 'ii' need to be updated */
                for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                {
                    u      = adjncy[jj];
                    other  = where[u];
                    orinfo = graph->vkrinfo + u;
                    onbrs  = ctrl->vnbrpool + orinfo->inbr;

                    for(kk = 0; kk < orinfo->nnbrs; kk++)
                    {
                        if(onbrs[kk].pid == to)
                        {
                            onbrs[kk].gv += vsize[ii];
                            if(!vmarker[u])
                            { /* Need to update boundary etc */
                                vmarker[u]     = 2;
                                modind[nmod++] = u;
                            }
                            break;
                        }
                    }
                }
            }
        }

        ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
    }


    /*======================================================================
   * Add the contributions on the volume gain due to 'v'
   *=====================================================================*/
    myrinfo = graph->vkrinfo + v;
    mynbrs  = ctrl->vnbrpool + myrinfo->inbr;
    for(k = 0; k < myrinfo->nnbrs; k++)
        pmarker[mynbrs[k].pid] = k;
    pmarker[to] = k;

    for(j = xadj[v]; j < xadj[v + 1]; j++)
    {
        ii     = adjncy[j];
        other  = where[ii];
        orinfo = graph->vkrinfo + ii;
        onbrs  = ctrl->vnbrpool + orinfo->inbr;

        if(other == to)
        {
            for(k = 0; k < orinfo->nnbrs; k++)
            {
                if(pmarker[onbrs[k].pid] == -1)
                    onbrs[k].gv -= vsize[v];
            }
        }
        else
        {
            ASSERT(pmarker[other] != -1);

            if(mynbrs[pmarker[other]].ned > 1)
            {
                for(k = 0; k < orinfo->nnbrs; k++)
                {
                    if(pmarker[onbrs[k].pid] == -1)
                        onbrs[k].gv -= vsize[v];
                }
            }
            else
            { /* There is only one connection */
                for(k = 0; k < orinfo->nnbrs; k++)
                {
                    if(pmarker[onbrs[k].pid] != -1)
                        onbrs[k].gv += vsize[v];
                }
            }
        }
    }
    for(k = 0; k < myrinfo->nnbrs; k++)
        pmarker[mynbrs[k].pid] = -1;
    pmarker[to] = -1;


    /*======================================================================
   * Recompute the volume information of the 'hard' nodes, and update the
   * max volume gain for all the modified vertices and the priority queue
   *=====================================================================*/
    for(iii = 0; iii < nmod; iii++)
    {
        i  = modind[iii];
        me = where[i];

        myrinfo = graph->vkrinfo + i;
        mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

        if(vmarker[i] == 1)
        { /* Only complete gain updates go through */
            for(k = 0; k < myrinfo->nnbrs; k++)
                mynbrs[k].gv = 0;

            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                ii     = adjncy[j];
                other  = where[ii];
                orinfo = graph->vkrinfo + ii;
                onbrs  = ctrl->vnbrpool + orinfo->inbr;

                for(kk = 0; kk < orinfo->nnbrs; kk++)
                    pmarker[onbrs[kk].pid] = kk;
                pmarker[other] = 1;

                if(me == other)
                {
                    /* Find which domains 'i' is connected and 'ii' is not and update their gain */
                    for(k = 0; k < myrinfo->nnbrs; k++)
                    {
                        if(pmarker[mynbrs[k].pid] == -1)
                            mynbrs[k].gv -= vsize[ii];
                    }
                }
                else
                {
                    ASSERT(pmarker[me] != -1);

                    /* I'm the only connection of 'ii' in 'me' */
                    if(onbrs[pmarker[me]].ned == 1)
                    {
                        /* Increase the gains for all the common domains between 'i' and 'ii' */
                        for(k = 0; k < myrinfo->nnbrs; k++)
                        {
                            if(pmarker[mynbrs[k].pid] != -1)
                                mynbrs[k].gv += vsize[ii];
                        }
                    }
                    else
                    {
                        /* Find which domains 'i' is connected and 'ii' is not and update their gain */
                        for(k = 0; k < myrinfo->nnbrs; k++)
                        {
                            if(pmarker[mynbrs[k].pid] == -1)
                                mynbrs[k].gv -= vsize[ii];
                        }
                    }
                }

                for(kk = 0; kk < orinfo->nnbrs; kk++)
                    pmarker[onbrs[kk].pid] = -1;
                pmarker[other] = -1;
            }
        }

        /* Compute the overall gv for that node */
        myrinfo->gv = IDX_MIN;
        for(k = 0; k < myrinfo->nnbrs; k++)
        {
            if(mynbrs[k].gv > myrinfo->gv)
                myrinfo->gv = mynbrs[k].gv;
        }

        /* Add the xtra gain due to id == 0 */
        if(myrinfo->ned > 0 && myrinfo->nid == 0)
            myrinfo->gv += vsize[i];


        /*======================================================================
     * Maintain a consistent boundary
     *=====================================================================*/
        if(bndtype == BNDTYPE_REFINE)
        {
            if(myrinfo->gv >= 0 && graph->bndptr[i] == -1)
                BNDInsert(graph->nbnd, graph->bndind, graph->bndptr, i);

            if(myrinfo->gv < 0 && graph->bndptr[i] != -1)
                BNDDelete(graph->nbnd, graph->bndind, graph->bndptr, i);
        }
        else
        {
            if(myrinfo->ned > 0 && graph->bndptr[i] == -1)
                BNDInsert(graph->nbnd, graph->bndind, graph->bndptr, i);

            if(myrinfo->ned == 0 && graph->bndptr[i] != -1)
                BNDDelete(graph->nbnd, graph->bndind, graph->bndptr, i);
        }


        /*======================================================================
     * Update the priority queue appropriately (if allowed)
     *=====================================================================*/
        if(queue != NULL)
        {
            if(vstatus[i] != VPQSTATUS_EXTRACTED)
            {
                if(graph->bndptr[i] != -1)
                { /* In-boundary vertex */
                    if(vstatus[i] == VPQSTATUS_PRESENT)
                    {
                        ipqUpdate(queue, i, myrinfo->gv);
                    }
                    else
                    {
                        ipqInsert(queue, i, myrinfo->gv);
                        vstatus[i] = VPQSTATUS_PRESENT;
                        ListInsert(*r_nupd, updind, updptr, i);
                    }
                }
                else
                { /* Off-boundary vertex */
                    if(vstatus[i] == VPQSTATUS_PRESENT)
                    {
                        ipqDelete(queue, i);
                        vstatus[i] = VPQSTATUS_NOTPRESENT;
                        ListDelete(*r_nupd, updind, updptr, i);
                    }
                }
            }
        }

        vmarker[i] = 0;
    }
}


/*************************************************************************/
/*! K-way partitioning optimization in which the vertices are visited in
    decreasing ed/sqrt(nnbrs)-id order. Note this is just an
    approximation, as the ed is often split across different subdomains
    and the sqrt(nnbrs) is just a crude approximation.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves
         to violate the max-pwgt constraint.
  \param omode is the type of optimization that will performed among
         OMODE_REFINE and OMODE_BALANCE


*/
/**************************************************************************/
void Greedy_KWayEdgeStats(ctrl_t* ctrl, graph_t* graph)
{
    /* Common variables to all types of kway-refinement/balancing routines */
    idx_t      i, ii, iii, j, k, l, nvtxs, nparts, gain, u, v, uw, vw;
    idx_t *    xadj, *adjncy, *adjwgt, *vwgt;
    idx_t *    where, *pwgts, *bndptr, *bndind, *minpwgts, *maxpwgts;
    idx_t      nbnd;
    ckrinfo_t *urinfo, *vrinfo;
    cnbr_t *   unbrs, *vnbrs;
    real_t *   tpwgts, ubfactor;

    WCOREPUSH;

    /* Link the graph fields */
    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vwgt   = graph->vwgt;
    adjwgt = graph->adjwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;

    where = graph->where;
    pwgts = graph->pwgts;

    nparts = ctrl->nparts;
    tpwgts = ctrl->tpwgts;

    /* Setup the weight intervals of the various subdomains */
    minpwgts = iwspacemalloc(ctrl, nparts);
    maxpwgts = iwspacemalloc(ctrl, nparts);

    ubfactor = ctrl->ubfactors[0];
    for(i = 0; i < nparts; i++)
    {
        maxpwgts[i] = tpwgts[i] * graph->tvwgt[0] * ubfactor;
        minpwgts[i] = tpwgts[i] * graph->tvwgt[0] * (0.95 / ubfactor);
    }

    /* go and determine the positive gain valid swaps */
    nbnd = graph->nbnd;

    for(ii = 0; ii < nbnd; ii++)
    {
        u  = bndind[ii];
        uw = where[u];

        urinfo = graph->ckrinfo + u;
        unbrs  = ctrl->cnbrpool + urinfo->inbr;

        for(j = xadj[u]; j < xadj[u + 1]; j++)
        {
            v  = adjncy[j];
            vw = where[v];

            vrinfo = graph->ckrinfo + v;
            vnbrs  = ctrl->cnbrpool + vrinfo->inbr;

            if(uw == vw)
                continue;
            if(pwgts[uw] - vwgt[u] + vwgt[v] > maxpwgts[uw]
               || pwgts[vw] - vwgt[v] + vwgt[u] > maxpwgts[vw])
                continue;

            for(k = urinfo->nnbrs - 1; k >= 0; k--)
            {
                if(unbrs[k].pid == vw)
                    break;
            }
            if(k < 0)
                printf("Something went wrong!\n");
            gain = unbrs[k].ed - urinfo->id;

            for(k = vrinfo->nnbrs - 1; k >= 0; k--)
            {
                if(vnbrs[k].pid == uw)
                    break;
            }
            if(k < 0)
                printf("Something went wrong!\n");
            gain += vnbrs[k].ed - vrinfo->id;

            gain -= 2 * adjwgt[j];

            if(gain > 0)
                printf("  Gain: %" PRIDX " for moving (%" PRIDX ", %" PRIDX
                       ") between (%" PRIDX ", %" PRIDX ")\n",
                       gain,
                       u,
                       v,
                       uw,
                       vw);
        }
    }

    WCOREPOP;
}


/*************************************************************************/
/*! K-way partitioning optimization in which the vertices are visited in
    random order and the best edge is selected to swap its incident vertices

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.

*/
/**************************************************************************/
void Greedy_KWayEdgeCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter)
{
    /* Common variables to all types of kway-refinement/balancing routines */
    idx_t   ii, j, k, pass, nvtxs, nparts, u, v, uw, vw, gain, bestgain, jbest;
    idx_t   from, me, to, oldcut, nmoved;
    idx_t * xadj, *adjncy, *adjwgt, *vwgt;
    idx_t * where, *pwgts, *perm, *bndptr, *bndind, *minpwgts, *maxpwgts;
    idx_t   bndtype = BNDTYPE_REFINE;
    real_t *tpwgts, ubfactor;

    /* Edgecut-specific/different variables */
    idx_t      nbnd, oldnnbrs;
    ckrinfo_t *myrinfo, *urinfo, *vrinfo;
    cnbr_t *   unbrs, *vnbrs;

    WCOREPUSH;

    /* Link the graph fields */
    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    vwgt   = graph->vwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;

    where = graph->where;
    pwgts = graph->pwgts;

    nparts = ctrl->nparts;
    tpwgts = ctrl->tpwgts;

    /* Setup the weight intervals of the various subdomains */
    minpwgts = iwspacemalloc(ctrl, nparts);
    maxpwgts = iwspacemalloc(ctrl, nparts);

    ubfactor = gk_max(ctrl->ubfactors[0], ComputeLoadImbalance(graph, nparts, ctrl->pijbm));
    for(k = 0; k < nparts; k++)
    {
        maxpwgts[k] = tpwgts[k] * graph->tvwgt[0] * ubfactor;
        minpwgts[k] = tpwgts[k] * graph->tvwgt[0] * (1.0 / ubfactor);
    }

    perm = iwspacemalloc(ctrl, nvtxs);


    if(ctrl->dbglvl & METIS_DBG_REFINE)
    {
        printf("GRE: [%6" PRIDX " %6" PRIDX "]-[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL
               ","
               " Nv-Nb[%6" PRIDX " %6" PRIDX "], Cut: %6" PRIDX "\n",
               pwgts[iargmin(nparts, pwgts, 1)],
               imax(nparts, pwgts, 1),
               minpwgts[0],
               maxpwgts[0],
               ComputeLoadImbalance(graph, nparts, ctrl->pijbm),
               graph->nvtxs,
               graph->nbnd,
               graph->mincut);
    }


    /*=====================================================================
  * The top-level refinement loop
  *======================================================================*/
    for(pass = 0; pass < niter; pass++)
    {
        GKASSERT(ComputeCut(graph, where) == graph->mincut);

        oldcut = graph->mincut;
        nbnd   = graph->nbnd;
        nmoved = 0;

        /* Insert the boundary vertices in the priority queue */
        /* Visit the vertices in random order and see if you can swap them */
        irandArrayPermute(nvtxs, perm, nbnd, 1);
        for(ii = 0; ii < nvtxs; ii++)
        {
            if(bndptr[u = perm[ii]] == -1)
                continue;

            uw = where[u];

            urinfo = graph->ckrinfo + u;
            unbrs  = ctrl->cnbrpool + urinfo->inbr;

            bestgain = 0;
            jbest    = -1;
            for(j = xadj[u]; j < xadj[u + 1]; j++)
            {
                v  = adjncy[j];
                vw = where[v];

                if(uw == vw)
                    continue;
                if(pwgts[uw] - vwgt[u] + vwgt[v] > maxpwgts[uw]
                   || pwgts[vw] - vwgt[v] + vwgt[u] > maxpwgts[vw])
                    continue;
                if(pwgts[uw] - vwgt[u] + vwgt[v] < minpwgts[uw]
                   || pwgts[vw] - vwgt[v] + vwgt[u] < minpwgts[vw])
                    continue;

                vrinfo = graph->ckrinfo + v;
                vnbrs  = ctrl->cnbrpool + vrinfo->inbr;

                gain = -2 * adjwgt[j];

                for(k = urinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(unbrs[k].pid == vw)
                        break;
                }
                GKASSERT(k >= 0);
                gain += unbrs[k].ed - urinfo->id;

                for(k = vrinfo->nnbrs - 1; k >= 0; k--)
                {
                    if(vnbrs[k].pid == uw)
                        break;
                }
                GKASSERT(k >= 0);
                gain += vnbrs[k].ed - vrinfo->id;

                if(gain > bestgain && vnbrs[k].ed > adjwgt[j])
                {
                    bestgain = gain;
                    jbest    = j;
                }
            }

            if(jbest == -1)
                continue; /* no valid positive swap */


            /*=====================================================================
      * If we got here, we can now swap the vertices
      *======================================================================*/
            v  = adjncy[jbest];
            vw = where[v];

            vrinfo = graph->ckrinfo + v;
            vnbrs  = ctrl->cnbrpool + vrinfo->inbr;

            /* move u to v's partition */
            for(k = urinfo->nnbrs - 1; k >= 0; k--)
            {
                if(unbrs[k].pid == vw)
                    break;
            }
            GKASSERT(k >= 0);

            from = uw;
            to   = vw;

            graph->mincut -= unbrs[k].ed - urinfo->id;
            nmoved++;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("\t\tMoving %6" PRIDX " from %3" PRIDX " to %3" PRIDX " [%6" PRIDX
                         " %6" PRIDX "]. Gain: %4" PRIDX ". Cut: %6" PRIDX "\n",
                         u,
                         from,
                         to,
                         pwgts[from],
                         pwgts[to],
                         unbrs[k].ed - urinfo->id,
                         graph->mincut));

            /* Update ID/ED and BND related information for the moved vertex */
            INC_DEC(pwgts[to], pwgts[from], vwgt[u]);
            UpdateMovedVertexInfoAndBND(
                u, from, k, to, urinfo, unbrs, where, nbnd, bndptr, bndind, bndtype);

            /* Update the degrees of adjacent vertices */
            for(j = xadj[u]; j < xadj[u + 1]; j++)
            {
                ii      = adjncy[j];
                me      = where[ii];
                myrinfo = graph->ckrinfo + ii;

                oldnnbrs = myrinfo->nnbrs;

                UpdateAdjacentVertexInfoAndBND(
                    ctrl, ii, xadj[ii + 1] - xadj[ii], me, from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

                ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
            }

            /* move v to u's partition */
            for(k = vrinfo->nnbrs - 1; k >= 0; k--)
            {
                if(vnbrs[k].pid == uw)
                    break;
            }
            GKASSERT(k >= 0);
#ifdef XXX
            if(k < 0)
            { /* that was removed, go and re-insert it */
                k            = vrinfo->nnbrs++;
                vnbrs[k].pid = uw;
                vnbrs[k].ed  = 0;
            }
#endif

            from = vw;
            to   = uw;

            graph->mincut -= vnbrs[k].ed - vrinfo->id;
            nmoved++;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("\t\tMoving %6" PRIDX " from %3" PRIDX " to %3" PRIDX " [%6" PRIDX
                         " %6" PRIDX "]. Gain: %4" PRIDX ". Cut: %6" PRIDX "\n",
                         v,
                         from,
                         to,
                         pwgts[from],
                         pwgts[to],
                         vnbrs[k].ed - vrinfo->id,
                         graph->mincut));

            /* Update ID/ED and BND related information for the moved vertex */
            INC_DEC(pwgts[to], pwgts[from], vwgt[v]);
            UpdateMovedVertexInfoAndBND(
                v, from, k, to, vrinfo, vnbrs, where, nbnd, bndptr, bndind, bndtype);

            /* Update the degrees of adjacent vertices */
            for(j = xadj[v]; j < xadj[v + 1]; j++)
            {
                ii      = adjncy[j];
                me      = where[ii];
                myrinfo = graph->ckrinfo + ii;

                oldnnbrs = myrinfo->nnbrs;

                UpdateAdjacentVertexInfoAndBND(
                    ctrl, ii, xadj[ii + 1] - xadj[ii], me, from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

                ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
            }
        }

        graph->nbnd = nbnd;

        if(ctrl->dbglvl & METIS_DBG_REFINE)
        {
            printf("\t[%6" PRIDX " %6" PRIDX "], Bal: %5.3" PRREAL ", Nb: %6" PRIDX
                   "."
                   " Nmoves: %5" PRIDX ", Cut: %6" PRIDX ", Vol: %6" PRIDX "\n",
                   pwgts[iargmin(nparts, pwgts, 1)],
                   imax(nparts, pwgts, 1),
                   ComputeLoadImbalance(graph, nparts, ctrl->pijbm),
                   graph->nbnd,
                   nmoved,
                   graph->mincut,
                   ComputeVolume(graph, where));
        }

        if(nmoved == 0 || graph->mincut == oldcut)
            break;
    }

    WCOREPOP;
}

/************************ kwayrefine.c ************************/
/*!
\file
\brief Driving routines for multilevel k-way refinement

\date   Started 7/28/1997
\author George
\author  Copyright 1997-2009, Regents of the University of Minnesota
\version $Id: kwayrefine.c 20398 2016-11-22 17:17:12Z karypis $
*/


/*************************************************************************/
/*! This function is the entry point of cut-based refinement */
/*************************************************************************/
void RefineKWay(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph)
{
    idx_t    i, nlevels, contig = ctrl->contig;
    graph_t* ptr;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->UncoarsenTmr));

    /* Determine how many levels are there */
    for(ptr = graph, nlevels = 0; ptr != orggraph; ptr = ptr->finer, nlevels++)
        ;

    /* Compute the parameters of the coarsest graph */
    ComputeKWayPartitionParams(ctrl, graph);

    /* Try to minimize the sub-domain connectivity */
    if(ctrl->minconn)
        EliminateSubDomainEdges(ctrl, graph);

    /* Deal with contiguity constraints at the beginning */
    if(contig && FindPartitionInducedComponents(graph, graph->where, NULL, NULL) > ctrl->nparts)
    {
        EliminateComponents(ctrl, graph);

        ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
        Greedy_KWayOptimize(ctrl, graph, 5, 0, OMODE_BALANCE);

        ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
        Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE);

        ctrl->contig = 0;
    }

    /* Refine each successively finer graph */
    for(i = 0;; i++)
    {
        if(ctrl->minconn && i == nlevels / 2)
            EliminateSubDomainEdges(ctrl, graph);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->RefTmr));

        if(2 * i >= nlevels && !IsBalanced(ctrl, graph, .02))
        {
            ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
            Greedy_KWayOptimize(ctrl, graph, 1, 0, OMODE_BALANCE);
            ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
        }

        Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 5.0, OMODE_REFINE);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->RefTmr));

        /* Deal with contiguity constraints in the middle */
        if(contig && i == nlevels / 2)
        {
            if(FindPartitionInducedComponents(graph, graph->where, NULL, NULL) > ctrl->nparts)
            {
                EliminateComponents(ctrl, graph);

                if(!IsBalanced(ctrl, graph, .02))
                {
                    ctrl->contig = 1;
                    ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
                    Greedy_KWayOptimize(ctrl, graph, 5, 0, OMODE_BALANCE);

                    ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
                    Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE);
                    ctrl->contig = 0;
                }
            }
        }

        if(graph == orggraph)
            break;

        graph = graph->finer;

        graph_ReadFromDisk(ctrl, graph);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ProjectTmr));
        ASSERT(graph->vwgt != NULL);

        ProjectKWayPartition(ctrl, graph);
        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ProjectTmr));
    }

    /* Deal with contiguity requirement at the end */
    ctrl->contig = contig;
    if(contig && FindPartitionInducedComponents(graph, graph->where, NULL, NULL) > ctrl->nparts)
        EliminateComponents(ctrl, graph);

    if(!IsBalanced(ctrl, graph, 0.0))
    {
        ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
        Greedy_KWayOptimize(ctrl, graph, 10, 0, OMODE_BALANCE);

        ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
        Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE);
    }

    if(ctrl->contig)
        ASSERT(FindPartitionInducedComponents(graph, graph->where, NULL, NULL) == ctrl->nparts);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->UncoarsenTmr));
}


/*************************************************************************/
/*! This function allocates memory for the k-way cut-based refinement */
/*************************************************************************/
void AllocateKWayPartitionMemory(ctrl_t* ctrl, graph_t* graph)
{

    graph->pwgts = imalloc(ctrl->nparts * graph->ncon, "AllocateKWayPartitionMemory: pwgts");
    graph->where = imalloc(graph->nvtxs, "AllocateKWayPartitionMemory: where");
    graph->bndptr = imalloc(graph->nvtxs, "AllocateKWayPartitionMemory: bndptr");
    graph->bndind = imalloc(graph->nvtxs, "AllocateKWayPartitionMemory: bndind");

    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT:
            graph->ckrinfo =
                (ckrinfo_t*)gk_malloc(graph->nvtxs * sizeof(ckrinfo_t),
                                      "AllocateKWayPartitionMemory: ckrinfo");
            break;

        case METIS_OBJTYPE_VOL:
            graph->vkrinfo =
                (vkrinfo_t*)gk_malloc(graph->nvtxs * sizeof(vkrinfo_t),
                                      "AllocateKWayVolPartitionMemory: vkrinfo");

            /* This is to let the cut-based -minconn and -contig large-scale graph
         changes to go through */
            graph->ckrinfo = (ckrinfo_t*)graph->vkrinfo;
            break;

        default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
    }
}


/*************************************************************************/
/*! This function computes the initial id/ed  for cut-based partitioning */
/*************************************************************************/
void ComputeKWayPartitionParams(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, j, k, l, nvtxs, ncon, nparts, nbnd, mincut, me, other;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *pwgts, *where, *bndind, *bndptr;

    nparts = ctrl->nparts;

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    where  = graph->where;
    pwgts  = iset(nparts * ncon, 0, graph->pwgts);
    bndind = graph->bndind;
    bndptr = iset(nvtxs, -1, graph->bndptr);

    nbnd = mincut = 0;

    /* Compute pwgts */
    if(ncon == 1)
    {
        for(i = 0; i < nvtxs; i++)
        {
            ASSERT(where[i] >= 0 && where[i] < nparts);
            pwgts[where[i]] += vwgt[i];
        }
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

    /* Compute the required info for refinement */
    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT: {
            ckrinfo_t* myrinfo;
            cnbr_t*    mynbrs;

            memset(graph->ckrinfo, 0, sizeof(ckrinfo_t) * nvtxs);
            cnbrpoolReset(ctrl);

            for(i = 0; i < nvtxs; i++)
            {
                me      = where[i];
                myrinfo = graph->ckrinfo + i;

                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    if(me == where[adjncy[j]])
                        myrinfo->id += adjwgt[j];
                    else
                        myrinfo->ed += adjwgt[j];
                }

                /* Time to compute the particular external degrees */
                if(myrinfo->ed > 0)
                {
                    mincut += myrinfo->ed;

                    myrinfo->inbr = cnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                    mynbrs = ctrl->cnbrpool + myrinfo->inbr;

                    for(j = xadj[i]; j < xadj[i + 1]; j++)
                    {
                        other = where[adjncy[j]];
                        if(me != other)
                        {
                            for(k = 0; k < myrinfo->nnbrs; k++)
                            {
                                if(mynbrs[k].pid == other)
                                {
                                    mynbrs[k].ed += adjwgt[j];
                                    break;
                                }
                            }
                            if(k == myrinfo->nnbrs)
                            {
                                mynbrs[k].pid = other;
                                mynbrs[k].ed  = adjwgt[j];
                                myrinfo->nnbrs++;
                            }
                        }
                    }

                    ASSERT(myrinfo->nnbrs <= xadj[i + 1] - xadj[i]);

                    /* Only ed-id>=0 nodes are considered to be in the boundary */
                    if(myrinfo->ed - myrinfo->id >= 0)
                        BNDInsert(nbnd, bndind, bndptr, i);
                }
                else
                {
                    myrinfo->inbr = -1;
                }
            }

            graph->mincut = mincut / 2;
            graph->nbnd   = nbnd;
        }
            ASSERT(CheckBnd2(graph));
            break;

        case METIS_OBJTYPE_VOL: {
            vkrinfo_t* myrinfo;
            vnbr_t*    mynbrs;

            memset(graph->vkrinfo, 0, sizeof(vkrinfo_t) * nvtxs);
            vnbrpoolReset(ctrl);

            /* Compute now the id/ed degrees */
            for(i = 0; i < nvtxs; i++)
            {
                me      = where[i];
                myrinfo = graph->vkrinfo + i;

                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    if(me == where[adjncy[j]])
                        myrinfo->nid++;
                    else
                        myrinfo->ned++;
                }

                /* Time to compute the particular external degrees */
                if(myrinfo->ned > 0)
                {
                    mincut += myrinfo->ned;

                    myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                    mynbrs = ctrl->vnbrpool + myrinfo->inbr;

                    for(j = xadj[i]; j < xadj[i + 1]; j++)
                    {
                        other = where[adjncy[j]];
                        if(me != other)
                        {
                            for(k = 0; k < myrinfo->nnbrs; k++)
                            {
                                if(mynbrs[k].pid == other)
                                {
                                    mynbrs[k].ned++;
                                    break;
                                }
                            }
                            if(k == myrinfo->nnbrs)
                            {
                                mynbrs[k].gv  = 0;
                                mynbrs[k].pid = other;
                                mynbrs[k].ned = 1;
                                myrinfo->nnbrs++;
                            }
                        }
                    }
                    ASSERT(myrinfo->nnbrs <= xadj[i + 1] - xadj[i]);
                }
                else
                {
                    myrinfo->inbr = -1;
                }
            }
            graph->mincut = mincut / 2;

            ComputeKWayVolGains(ctrl, graph);
        }
            ASSERT(graph->minvol == ComputeVolume(graph, graph->where));
            break;
        default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
    }
}


/*************************************************************************/
/*! This function projects a partition, and at the same time computes the
 parameters for refinement. */
/*************************************************************************/
void ProjectKWayPartition(ctrl_t* ctrl, graph_t* graph)
{
    idx_t    i, j, k, nvtxs, nbnd, nparts, me, other, istart, iend, tid, ted;
    idx_t *  xadj, *adjncy, *adjwgt;
    idx_t *  cmap, *where, *bndptr, *bndind, *cwhere, *htable;
    graph_t* cgraph;
    int      dropedges;

    WCOREPUSH;

    dropedges = ctrl->dropedges;

    nparts = ctrl->nparts;

    cgraph = graph->coarser;
    cwhere = cgraph->where;

    if(ctrl->objtype == METIS_OBJTYPE_CUT)
    {
        ASSERT(CheckBnd2(cgraph));
    }
    else
    {
        ASSERT(cgraph->minvol == ComputeVolume(cgraph, cgraph->where));
    }

    /* free the coarse graph's structure (reduce maxmem) */
    FreeSData(cgraph);

    nvtxs  = graph->nvtxs;
    cmap   = graph->cmap;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    AllocateKWayPartitionMemory(ctrl, graph);

    where  = graph->where;
    bndind = graph->bndind;
    bndptr = iset(nvtxs, -1, graph->bndptr);

    htable = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

    /* Compute the required info for refinement */
    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT: {
            ckrinfo_t* myrinfo;
            cnbr_t*    mynbrs;

            /* go through and project partition and compute id/ed for the nodes */
            for(i = 0; i < nvtxs; i++)
            {
                k        = cmap[i];
                where[i] = cwhere[k];
                cmap[i] = (dropedges ? 1 : cgraph->ckrinfo[k].ed); /* For optimization */
            }

            memset(graph->ckrinfo, 0, sizeof(ckrinfo_t) * nvtxs);
            cnbrpoolReset(ctrl);

            for(nbnd = 0, i = 0; i < nvtxs; i++)
            {
                istart = xadj[i];
                iend   = xadj[i + 1];

                myrinfo = graph->ckrinfo + i;

                if(cmap[i] == 0)
                { /* Interior node. Note that cmap[i] = crinfo[cmap[i]].ed */
                    for(tid = 0, j = istart; j < iend; j++)
                        tid += adjwgt[j];

                    myrinfo->id   = tid;
                    myrinfo->inbr = -1;
                }
                else
                { /* Potentially an interface node */
                    myrinfo->inbr = cnbrpoolGetNext(ctrl, iend - istart);
                    mynbrs        = ctrl->cnbrpool + myrinfo->inbr;

                    me = where[i];
                    for(tid = 0, ted = 0, j = istart; j < iend; j++)
                    {
                        other = where[adjncy[j]];
                        if(me == other)
                        {
                            tid += adjwgt[j];
                        }
                        else
                        {
                            ted += adjwgt[j];
                            if((k = htable[other]) == -1)
                            {
                                htable[other]               = myrinfo->nnbrs;
                                mynbrs[myrinfo->nnbrs].pid  = other;
                                mynbrs[myrinfo->nnbrs++].ed = adjwgt[j];
                            }
                            else
                            {
                                mynbrs[k].ed += adjwgt[j];
                            }
                        }
                    }
                    myrinfo->id = tid;
                    myrinfo->ed = ted;

                    /* Remove space for edegrees if it was interior */
                    if(ted == 0)
                    {
                        ctrl->nbrpoolcpos -= gk_min(nparts, iend - istart);
                        myrinfo->inbr = -1;
                    }
                    else
                    {
                        if(ted - tid >= 0)
                            BNDInsert(nbnd, bndind, bndptr, i);

                        for(j = 0; j < myrinfo->nnbrs; j++)
                            htable[mynbrs[j].pid] = -1;
                    }
                }
            }

            graph->nbnd = nbnd;
        }
            ASSERT(CheckBnd2(graph));
            break;

        case METIS_OBJTYPE_VOL: {
            vkrinfo_t* myrinfo;
            vnbr_t*    mynbrs;

            /* go through and project partition and compute id/ed for the nodes */
            for(i = 0; i < nvtxs; i++)
            {
                k        = cmap[i];
                where[i] = cwhere[k];
                cmap[i] = (dropedges ? 1 : cgraph->vkrinfo[k].ned); /* For optimization */
            }

            memset(graph->vkrinfo, 0, sizeof(vkrinfo_t) * nvtxs);
            vnbrpoolReset(ctrl);

            for(i = 0; i < nvtxs; i++)
            {
                istart  = xadj[i];
                iend    = xadj[i + 1];
                myrinfo = graph->vkrinfo + i;

                if(cmap[i] == 0)
                { /* Note that cmap[i] = crinfo[cmap[i]].ed */
                    myrinfo->nid  = iend - istart;
                    myrinfo->inbr = -1;
                }
                else
                { /* Potentially an interface node */
                    myrinfo->inbr = vnbrpoolGetNext(ctrl, iend - istart);
                    mynbrs        = ctrl->vnbrpool + myrinfo->inbr;

                    me = where[i];
                    for(tid = 0, ted = 0, j = istart; j < iend; j++)
                    {
                        other = where[adjncy[j]];
                        if(me == other)
                        {
                            tid++;
                        }
                        else
                        {
                            ted++;
                            if((k = htable[other]) == -1)
                            {
                                htable[other]                = myrinfo->nnbrs;
                                mynbrs[myrinfo->nnbrs].gv    = 0;
                                mynbrs[myrinfo->nnbrs].pid   = other;
                                mynbrs[myrinfo->nnbrs++].ned = 1;
                            }
                            else
                            {
                                mynbrs[k].ned++;
                            }
                        }
                    }
                    myrinfo->nid = tid;
                    myrinfo->ned = ted;

                    /* Remove space for edegrees if it was interior */
                    if(ted == 0)
                    {
                        ctrl->nbrpoolcpos -= gk_min(nparts, iend - istart);
                        myrinfo->inbr = -1;
                    }
                    else
                    {
                        for(j = 0; j < myrinfo->nnbrs; j++)
                            htable[mynbrs[j].pid] = -1;
                    }
                }
            }

            ComputeKWayVolGains(ctrl, graph);

            ASSERT(graph->minvol == ComputeVolume(graph, graph->where));
        }
        break;

        default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
    }

    graph->mincut = (dropedges ? ComputeCut(graph, where) : cgraph->mincut);
    icopy(nparts * graph->ncon, cgraph->pwgts, graph->pwgts);

    FreeGraph(&graph->coarser);

    WCOREPOP;
}


/*************************************************************************/
/*! This function computes the boundary definition for balancing. */
/*************************************************************************/
void ComputeKWayBoundary(ctrl_t* ctrl, graph_t* graph, idx_t bndtype)
{
    idx_t  i, nvtxs, nbnd;
    idx_t *bndind, *bndptr;

    nvtxs  = graph->nvtxs;
    bndind = graph->bndind;
    bndptr = iset(nvtxs, -1, graph->bndptr);

    nbnd = 0;

    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT:
            /* Compute the boundary */
            if(bndtype == BNDTYPE_REFINE)
            {
                for(i = 0; i < nvtxs; i++)
                {
                    if(graph->ckrinfo[i].ed > 0
                       && graph->ckrinfo[i].ed - graph->ckrinfo[i].id >= 0)
                        BNDInsert(nbnd, bndind, bndptr, i);
                }
            }
            else
            { /* BNDTYPE_BALANCE */
                for(i = 0; i < nvtxs; i++)
                {
                    if(graph->ckrinfo[i].ed > 0)
                        BNDInsert(nbnd, bndind, bndptr, i);
                }
            }
            break;

        case METIS_OBJTYPE_VOL:
            /* Compute the boundary */
            if(bndtype == BNDTYPE_REFINE)
            {
                for(i = 0; i < nvtxs; i++)
                {
                    if(graph->vkrinfo[i].gv >= 0)
                        BNDInsert(nbnd, bndind, bndptr, i);
                }
            }
            else
            { /* BNDTYPE_BALANCE */
                for(i = 0; i < nvtxs; i++)
                {
                    if(graph->vkrinfo[i].ned > 0)
                        BNDInsert(nbnd, bndind, bndptr, i);
                }
            }
            break;

        default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
    }

    graph->nbnd = nbnd;
}


/*************************************************************************/
/*! This function computes the initial gains in the communication volume */
/*************************************************************************/
void ComputeKWayVolGains(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, ii, j, k, l, nvtxs, nparts, me, other, pid;
    idx_t *xadj, *vsize, *adjncy, *adjwgt, *where, *bndind, *bndptr, *ophtable;
    vkrinfo_t *myrinfo, *orinfo;
    vnbr_t *   mynbrs, *onbrs;

    WCOREPUSH;

    nparts = ctrl->nparts;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vsize  = graph->vsize;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    where  = graph->where;
    bndind = graph->bndind;
    bndptr = iset(nvtxs, -1, graph->bndptr);

    ophtable = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

    /* Compute the volume gains */
    graph->minvol = graph->nbnd = 0;
    for(i = 0; i < nvtxs; i++)
    {
        myrinfo     = graph->vkrinfo + i;
        myrinfo->gv = IDX_MIN;

        if(myrinfo->nnbrs > 0)
        {
            me     = where[i];
            mynbrs = ctrl->vnbrpool + myrinfo->inbr;

            graph->minvol += myrinfo->nnbrs * vsize[i];

            for(j = xadj[i]; j < xadj[i + 1]; j++)
            {
                ii     = adjncy[j];
                other  = where[ii];
                orinfo = graph->vkrinfo + ii;
                onbrs  = ctrl->vnbrpool + orinfo->inbr;

                for(k = 0; k < orinfo->nnbrs; k++)
                    ophtable[onbrs[k].pid] = k;
                ophtable[other] = 1; /* this is to simplify coding */

                if(me == other)
                {
                    /* Find which domains 'i' is connected to but 'ii' is not
             and update their gain */
                    for(k = 0; k < myrinfo->nnbrs; k++)
                    {
                        if(ophtable[mynbrs[k].pid] == -1)
                            mynbrs[k].gv -= vsize[ii];
                    }
                }
                else
                {
                    ASSERT(ophtable[me] != -1);

                    if(onbrs[ophtable[me]].ned == 1)
                    {
                        /* I'm the only connection of 'ii' in 'me' */
                        /* Increase the gains for all the common domains between 'i' and 'ii' */
                        for(k = 0; k < myrinfo->nnbrs; k++)
                        {
                            if(ophtable[mynbrs[k].pid] != -1)
                                mynbrs[k].gv += vsize[ii];
                        }
                    }
                    else
                    {
                        /* Find which domains 'i' is connected to and 'ii' is not
               and update their gain */
                        for(k = 0; k < myrinfo->nnbrs; k++)
                        {
                            if(ophtable[mynbrs[k].pid] == -1)
                                mynbrs[k].gv -= vsize[ii];
                        }
                    }
                }

                /* Reset the marker vector */
                for(k = 0; k < orinfo->nnbrs; k++)
                    ophtable[onbrs[k].pid] = -1;
                ophtable[other] = -1;
            }

            /* Compute the max vgain */
            for(k = 0; k < myrinfo->nnbrs; k++)
            {
                if(mynbrs[k].gv > myrinfo->gv)
                    myrinfo->gv = mynbrs[k].gv;
            }

            /* Add the extra gain due to id == 0 */
            if(myrinfo->ned > 0 && myrinfo->nid == 0)
                myrinfo->gv += vsize[i];
        }

        if(myrinfo->gv >= 0)
            BNDInsert(graph->nbnd, bndind, bndptr, i);
    }

    WCOREPOP;
}


/*************************************************************************/
/*! This function checks if the partition weights are within the balance
constraints */
/*************************************************************************/
int IsBalanced(ctrl_t* ctrl, graph_t* graph, real_t ffactor)
{
    return (ComputeLoadImbalanceDiff(graph, ctrl->nparts, ctrl->pijbm, ctrl->ubfactors)
            <= ffactor);
}

/************************ sfm.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * sfm.c
 *
 * This file contains code that implements an FM-based separator refinement
 *
 * Started 8/1/97
 * George
 *
 * $Id: sfm.c 10874 2011-10-17 23:13:00Z karypis $
 *
 */


/*************************************************************************/
/*! This function performs a node-based FM refinement */
/**************************************************************************/
void FM_2WayNodeRefine2Sided(ctrl_t* ctrl, graph_t* graph, idx_t niter)
{
    idx_t     i, ii, j, k, jj, kk, nvtxs, nbnd, nswaps, nmind;
    idx_t *   xadj, *vwgt, *adjncy, *where, *pwgts, *edegrees, *bndind, *bndptr;
    idx_t *   mptr, *mind, *moved, *swaps;
    rpq_t*    queues[2];
    nrinfo_t* rinfo;
    idx_t     higain, oldgain, mincut, initcut, mincutorder;
    idx_t     pass, to, other, limit;
    idx_t     badmaxpwgt, mindiff, newdiff;
    idx_t     u[2], g[2];
    real_t    mult;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vwgt   = graph->vwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;
    where  = graph->where;
    pwgts  = graph->pwgts;
    rinfo  = graph->nrinfo;

    queues[0] = rpqCreate(nvtxs);
    queues[1] = rpqCreate(nvtxs);

    moved = iwspacemalloc(ctrl, nvtxs);
    swaps = iwspacemalloc(ctrl, nvtxs);
    mptr  = iwspacemalloc(ctrl, nvtxs + 1);
    mind  = iwspacemalloc(ctrl, 2 * nvtxs);

    mult       = 0.5 * ctrl->ubfactors[0];
    badmaxpwgt = (idx_t)(mult * (pwgts[0] + pwgts[1] + pwgts[2]));

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("Partitions-N2: [%6" PRIDX " %6" PRIDX "] Nv-Nb[%6" PRIDX
                 " %6" PRIDX "]. ISep: %6" PRIDX "\n",
                 pwgts[0],
                 pwgts[1],
                 graph->nvtxs,
                 graph->nbnd,
                 graph->mincut));

    for(pass = 0; pass < niter; pass++)
    {
        iset(nvtxs, -1, moved);
        rpqReset(queues[0]);
        rpqReset(queues[1]);

        mincutorder = -1;
        initcut = mincut = graph->mincut;
        nbnd             = graph->nbnd;

        /* use the swaps array in place of the traditional perm array to save memory */
        irandArrayPermute(nbnd, swaps, nbnd, 1);
        for(ii = 0; ii < nbnd; ii++)
        {
            i = bndind[swaps[ii]];
            ASSERT(where[i] == 2);
            rpqInsert(queues[0], i, vwgt[i] - rinfo[i].edegrees[1]);
            rpqInsert(queues[1], i, vwgt[i] - rinfo[i].edegrees[0]);
        }

        ASSERT(CheckNodeBnd(graph, nbnd));
        ASSERT(CheckNodePartitionParams(graph));

        limit = (ctrl->compress ? gk_min(5 * nbnd, 400) : gk_min(2 * nbnd, 300));

        /******************************************************
    * Get into the FM loop
    *******************************************************/
        mptr[0] = nmind = 0;
        mindiff         = iabs(pwgts[0] - pwgts[1]);
        to              = (pwgts[0] < pwgts[1] ? 0 : 1);
        for(nswaps = 0; nswaps < nvtxs; nswaps++)
        {
            u[0] = rpqSeeTopVal(queues[0]);
            u[1] = rpqSeeTopVal(queues[1]);
            if(u[0] != -1 && u[1] != -1)
            {
                g[0] = vwgt[u[0]] - rinfo[u[0]].edegrees[1];
                g[1] = vwgt[u[1]] - rinfo[u[1]].edegrees[0];

                to = (g[0] > g[1] ? 0 : (g[0] < g[1] ? 1 : pass % 2));

                if(pwgts[to] + vwgt[u[to]] > badmaxpwgt)
                    to = (to + 1) % 2;
            }
            else if(u[0] == -1 && u[1] == -1)
            {
                break;
            }
            else if(u[0] != -1 && pwgts[0] + vwgt[u[0]] <= badmaxpwgt)
            {
                to = 0;
            }
            else if(u[1] != -1 && pwgts[1] + vwgt[u[1]] <= badmaxpwgt)
            {
                to = 1;
            }
            else
                break;

            other = (to + 1) % 2;

            higain = rpqGetTop(queues[to]);
            if(moved[higain] == -1) /* Delete if it was in the separator originally */
                rpqDelete(queues[other], higain);

            ASSERT(bndptr[higain] != -1);

            /* The following check is to ensure we break out if there is a possibility
         of over-running the mind array.  */
            if(nmind + xadj[higain + 1] - xadj[higain] >= 2 * nvtxs - 1)
                break;

            pwgts[2] -= (vwgt[higain] - rinfo[higain].edegrees[other]);

            newdiff = iabs(pwgts[to] + vwgt[higain]
                           - (pwgts[other] - rinfo[higain].edegrees[other]));
            if(pwgts[2] < mincut || (pwgts[2] == mincut && newdiff < mindiff))
            {
                mincut      = pwgts[2];
                mincutorder = nswaps;
                mindiff     = newdiff;
            }
            else
            {
                if(nswaps - mincutorder > 2 * limit
                   || (nswaps - mincutorder > limit && pwgts[2] > 1.10 * mincut))
                {
                    pwgts[2] += (vwgt[higain] - rinfo[higain].edegrees[other]);
                    break; /* No further improvement, break out */
                }
            }

            BNDDelete(nbnd, bndind, bndptr, higain);
            pwgts[to] += vwgt[higain];
            where[higain] = to;
            moved[higain] = nswaps;
            swaps[nswaps] = higain;


            /**********************************************************
      * Update the degrees of the affected nodes
      ***********************************************************/
            for(j = xadj[higain]; j < xadj[higain + 1]; j++)
            {
                k = adjncy[j];
                if(where[k] == 2)
                { /* For the in-separator vertices modify their edegree[to] */
                    oldgain = vwgt[k] - rinfo[k].edegrees[to];
                    rinfo[k].edegrees[to] += vwgt[higain];
                    if(moved[k] == -1 || moved[k] == -(2 + other))
                        rpqUpdate(queues[other], k, oldgain - vwgt[higain]);
                }
                else if(where[k] == other)
                { /* This vertex is pulled into the separator */
                    ASSERTP(bndptr[k] == -1,
                            ("%" PRIDX " %" PRIDX " %" PRIDX "\n", k, bndptr[k], where[k]));
                    BNDInsert(nbnd, bndind, bndptr, k);

                    mind[nmind++] = k; /* Keep track for rollback */
                    where[k]      = 2;
                    pwgts[other] -= vwgt[k];

                    edegrees    = rinfo[k].edegrees;
                    edegrees[0] = edegrees[1] = 0;
                    for(jj = xadj[k]; jj < xadj[k + 1]; jj++)
                    {
                        kk = adjncy[jj];
                        if(where[kk] != 2)
                            edegrees[where[kk]] += vwgt[kk];
                        else
                        {
                            oldgain = vwgt[kk] - rinfo[kk].edegrees[other];
                            rinfo[kk].edegrees[other] -= vwgt[k];
                            if(moved[kk] == -1 || moved[kk] == -(2 + to))
                                rpqUpdate(queues[to], kk, oldgain + vwgt[k]);
                        }
                    }

                    /* Insert the new vertex into the priority queue. Only one side! */
                    if(moved[k] == -1)
                    {
                        rpqInsert(queues[to], k, vwgt[k] - edegrees[other]);
                        moved[k] = -(2 + to);
                    }
                }
            }
            mptr[nswaps + 1] = nmind;

            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("Moved %6" PRIDX " to %3" PRIDX ", Gain: %5" PRIDX " [%5" PRIDX "] [%4" PRIDX
                         " %4" PRIDX "] \t[%5" PRIDX " %5" PRIDX " %5" PRIDX "]\n",
                         higain,
                         to,
                         g[to],
                         g[other],
                         vwgt[u[to]],
                         vwgt[u[other]],
                         pwgts[0],
                         pwgts[1],
                         pwgts[2]));
        }


        /****************************************************************
    * Roll back computation
    *****************************************************************/
        for(nswaps--; nswaps > mincutorder; nswaps--)
        {
            higain = swaps[nswaps];

            ASSERT(CheckNodePartitionParams(graph));

            to    = where[higain];
            other = (to + 1) % 2;
            INC_DEC(pwgts[2], pwgts[to], vwgt[higain]);
            where[higain] = 2;
            BNDInsert(nbnd, bndind, bndptr, higain);

            edegrees    = rinfo[higain].edegrees;
            edegrees[0] = edegrees[1] = 0;
            for(j = xadj[higain]; j < xadj[higain + 1]; j++)
            {
                k = adjncy[j];
                if(where[k] == 2)
                    rinfo[k].edegrees[to] -= vwgt[higain];
                else
                    edegrees[where[k]] += vwgt[k];
            }

            /* Push nodes out of the separator */
            for(j = mptr[nswaps]; j < mptr[nswaps + 1]; j++)
            {
                k = mind[j];
                ASSERT(where[k] == 2);
                where[k] = other;
                INC_DEC(pwgts[other], pwgts[2], vwgt[k]);
                BNDDelete(nbnd, bndind, bndptr, k);
                for(jj = xadj[k]; jj < xadj[k + 1]; jj++)
                {
                    kk = adjncy[jj];
                    if(where[kk] == 2)
                        rinfo[kk].edegrees[other] += vwgt[k];
                }
            }
        }

        ASSERT(mincut == pwgts[2]);

        IFSET(ctrl->dbglvl,
              METIS_DBG_REFINE,
              printf("\tMinimum sep: %6" PRIDX " at %5" PRIDX
                     ", PWGTS: [%6" PRIDX " %6" PRIDX "], NBND: %6" PRIDX "\n",
                     mincut,
                     mincutorder,
                     pwgts[0],
                     pwgts[1],
                     nbnd));

        graph->mincut = mincut;
        graph->nbnd   = nbnd;

        if(mincutorder == -1 || mincut >= initcut)
            break;
    }

    rpqDestroy(queues[0]);
    rpqDestroy(queues[1]);

    WCOREPOP;
}


/*************************************************************************/
/*! This function performs a node-based FM refinement.
    Each refinement iteration is split into two sub-iterations.
    In each sub-iteration only moves to one of the left/right partitions
    is allowed; hence, it is one-sided.
*/
/**************************************************************************/
void FM_2WayNodeRefine1Sided(ctrl_t* ctrl, graph_t* graph, idx_t niter)
{
    idx_t     i, ii, j, k, jj, kk, nvtxs, nbnd, nswaps, nmind, iend;
    idx_t *   xadj, *vwgt, *adjncy, *where, *pwgts, *edegrees, *bndind, *bndptr;
    idx_t *   mptr, *mind, *swaps;
    rpq_t*    queue;
    nrinfo_t* rinfo;
    idx_t     higain, mincut, initcut, mincutorder;
    idx_t     pass, to, other, limit;
    idx_t     badmaxpwgt, mindiff, newdiff;
    real_t    mult;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vwgt   = graph->vwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;
    where  = graph->where;
    pwgts  = graph->pwgts;
    rinfo  = graph->nrinfo;

    queue = rpqCreate(nvtxs);

    swaps = iwspacemalloc(ctrl, nvtxs);
    mptr  = iwspacemalloc(ctrl, nvtxs + 1);
    mind  = iwspacemalloc(ctrl, 2 * nvtxs);

    mult       = 0.5 * ctrl->ubfactors[0];
    badmaxpwgt = (idx_t)(mult * (pwgts[0] + pwgts[1] + pwgts[2]));

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("Partitions-N1: [%6" PRIDX " %6" PRIDX "] Nv-Nb[%6" PRIDX
                 " %6" PRIDX "]. ISep: %6" PRIDX "\n",
                 pwgts[0],
                 pwgts[1],
                 graph->nvtxs,
                 graph->nbnd,
                 graph->mincut));

    to = (pwgts[0] < pwgts[1] ? 1 : 0);
    for(pass = 0; pass < 2 * niter; pass++)
    { /* the 2*niter is for the two sides */
        other = to;
        to    = (to + 1) % 2;

        rpqReset(queue);

        mincutorder = -1;
        initcut = mincut = graph->mincut;
        nbnd             = graph->nbnd;

        /* use the swaps array in place of the traditional perm array to save memory */
        irandArrayPermute(nbnd, swaps, nbnd, 1);
        for(ii = 0; ii < nbnd; ii++)
        {
            i = bndind[swaps[ii]];
            ASSERT(where[i] == 2);
            rpqInsert(queue, i, vwgt[i] - rinfo[i].edegrees[other]);
        }

        ASSERT(CheckNodeBnd(graph, nbnd));
        ASSERT(CheckNodePartitionParams(graph));

        limit = (ctrl->compress ? gk_min(5 * nbnd, 500) : gk_min(3 * nbnd, 300));

        /******************************************************
    * Get into the FM loop
    *******************************************************/
        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux3Tmr));
        mptr[0] = nmind = 0;
        mindiff         = iabs(pwgts[0] - pwgts[1]);
        for(nswaps = 0; nswaps < nvtxs; nswaps++)
        {
            if((higain = rpqGetTop(queue)) == -1)
                break;

            ASSERT(bndptr[higain] != -1);

            /* The following check is to ensure we break out if there is a possibility
         of over-running the mind array.  */
            if(nmind + xadj[higain + 1] - xadj[higain] >= 2 * nvtxs - 1)
                break;

            if(pwgts[to] + vwgt[higain] > badmaxpwgt)
                break; /* No point going any further. Balance will be bad */

            pwgts[2] -= (vwgt[higain] - rinfo[higain].edegrees[other]);

            newdiff = iabs(pwgts[to] + vwgt[higain]
                           - (pwgts[other] - rinfo[higain].edegrees[other]));
            if(pwgts[2] < mincut || (pwgts[2] == mincut && newdiff < mindiff))
            {
                mincut      = pwgts[2];
                mincutorder = nswaps;
                mindiff     = newdiff;
            }
            else
            {
                if(nswaps - mincutorder > 3 * limit
                   || (nswaps - mincutorder > limit && pwgts[2] > 1.10 * mincut))
                {
                    pwgts[2] += (vwgt[higain] - rinfo[higain].edegrees[other]);
                    break; /* No further improvement, break out */
                }
            }

            BNDDelete(nbnd, bndind, bndptr, higain);
            pwgts[to] += vwgt[higain];
            where[higain] = to;
            swaps[nswaps] = higain;


            /**********************************************************
      * Update the degrees of the affected nodes
      ***********************************************************/
            IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux1Tmr));
            for(j = xadj[higain]; j < xadj[higain + 1]; j++)
            {
                k = adjncy[j];

                if(where[k] == 2)
                { /* For the in-separator vertices modify their edegree[to] */
                    rinfo[k].edegrees[to] += vwgt[higain];
                }
                else if(where[k] == other)
                { /* This vertex is pulled into the separator */
                    ASSERTP(bndptr[k] == -1,
                            ("%" PRIDX " %" PRIDX " %" PRIDX "\n", k, bndptr[k], where[k]));
                    BNDInsert(nbnd, bndind, bndptr, k);

                    mind[nmind++] = k; /* Keep track for rollback */
                    where[k]      = 2;
                    pwgts[other] -= vwgt[k];

                    edegrees    = rinfo[k].edegrees;
                    edegrees[0] = edegrees[1] = 0;
                    for(jj = xadj[k], iend = xadj[k + 1]; jj < iend; jj++)
                    {
                        kk = adjncy[jj];
                        if(where[kk] != 2)
                            edegrees[where[kk]] += vwgt[kk];
                        else
                        {
                            rinfo[kk].edegrees[other] -= vwgt[k];

                            /* Since the moves are one-sided this vertex has not been moved yet */
                            rpqUpdate(queue, kk, vwgt[kk] - rinfo[kk].edegrees[other]);
                        }
                    }

                    /* Insert the new vertex into the priority queue. Safe due to one-sided moves */
                    rpqInsert(queue, k, vwgt[k] - edegrees[other]);
                }
            }
            mptr[nswaps + 1] = nmind;
            IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux1Tmr));


            IFSET(ctrl->dbglvl,
                  METIS_DBG_MOVEINFO,
                  printf("Moved %6" PRIDX " to %3" PRIDX ", Gain: %5" PRIDX " [%5" PRIDX "] \t[%5" PRIDX
                         " %5" PRIDX " %5" PRIDX "] [%3" PRIDX " %2" PRIDX "]\n",
                         higain,
                         to,
                         (vwgt[higain] - rinfo[higain].edegrees[other]),
                         vwgt[higain],
                         pwgts[0],
                         pwgts[1],
                         pwgts[2],
                         nswaps,
                         limit));
        }
        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux3Tmr));


        /****************************************************************
    * Roll back computation
    *****************************************************************/
        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux2Tmr));
        for(nswaps--; nswaps > mincutorder; nswaps--)
        {
            higain = swaps[nswaps];

            ASSERT(CheckNodePartitionParams(graph));
            ASSERT(where[higain] == to);

            INC_DEC(pwgts[2], pwgts[to], vwgt[higain]);
            where[higain] = 2;
            BNDInsert(nbnd, bndind, bndptr, higain);

            edegrees    = rinfo[higain].edegrees;
            edegrees[0] = edegrees[1] = 0;
            for(j = xadj[higain]; j < xadj[higain + 1]; j++)
            {
                k = adjncy[j];
                if(where[k] == 2)
                    rinfo[k].edegrees[to] -= vwgt[higain];
                else
                    edegrees[where[k]] += vwgt[k];
            }

            /* Push nodes out of the separator */
            for(j = mptr[nswaps]; j < mptr[nswaps + 1]; j++)
            {
                k = mind[j];
                ASSERT(where[k] == 2);
                where[k] = other;
                INC_DEC(pwgts[other], pwgts[2], vwgt[k]);
                BNDDelete(nbnd, bndind, bndptr, k);
                for(jj = xadj[k], iend = xadj[k + 1]; jj < iend; jj++)
                {
                    kk = adjncy[jj];
                    if(where[kk] == 2)
                        rinfo[kk].edegrees[other] += vwgt[k];
                }
            }
        }
        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux2Tmr));

        ASSERT(mincut == pwgts[2]);

        IFSET(ctrl->dbglvl,
              METIS_DBG_REFINE,
              printf("\tMinimum sep: %6" PRIDX " at %5" PRIDX
                     ", PWGTS: [%6" PRIDX " %6" PRIDX "], NBND: %6" PRIDX "\n",
                     mincut,
                     mincutorder,
                     pwgts[0],
                     pwgts[1],
                     nbnd));

        graph->mincut = mincut;
        graph->nbnd   = nbnd;

        if(pass % 2 == 1 && (mincutorder == -1 || mincut >= initcut))
            break;
    }

    rpqDestroy(queue);

    WCOREPOP;
}


/*************************************************************************/
/*! This function balances the left/right partitions of a separator
    tri-section */
/*************************************************************************/
void FM_2WayNodeBalance(ctrl_t* ctrl, graph_t* graph)
{
    idx_t     i, ii, j, k, jj, kk, nvtxs, nbnd, nswaps, gain;
    idx_t     badmaxpwgt, higain, oldgain, pass, to, other;
    idx_t *   xadj, *vwgt, *adjncy, *where, *pwgts, *edegrees, *bndind, *bndptr;
    idx_t *   perm, *moved;
    rpq_t*    queue;
    nrinfo_t* rinfo;
    real_t    mult;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vwgt   = graph->vwgt;

    bndind = graph->bndind;
    bndptr = graph->bndptr;
    where  = graph->where;
    pwgts  = graph->pwgts;
    rinfo  = graph->nrinfo;

    mult = 0.5 * ctrl->ubfactors[0];

    badmaxpwgt = (idx_t)(mult * (pwgts[0] + pwgts[1]));
    if(gk_max(pwgts[0], pwgts[1]) < badmaxpwgt)
        return;
    if(iabs(pwgts[0] - pwgts[1]) < 3 * graph->tvwgt[0] / nvtxs)
        return;

    WCOREPUSH;

    to    = (pwgts[0] < pwgts[1] ? 0 : 1);
    other = (to + 1) % 2;

    queue = rpqCreate(nvtxs);

    perm  = iwspacemalloc(ctrl, nvtxs);
    moved = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("Partitions: [%6" PRIDX " %6" PRIDX "] Nv-Nb[%6" PRIDX
                 " %6" PRIDX "]. ISep: %6" PRIDX " [B]\n",
                 pwgts[0],
                 pwgts[1],
                 graph->nvtxs,
                 graph->nbnd,
                 graph->mincut));

    nbnd = graph->nbnd;
    irandArrayPermute(nbnd, perm, nbnd, 1);
    for(ii = 0; ii < nbnd; ii++)
    {
        i = bndind[perm[ii]];
        ASSERT(where[i] == 2);
        rpqInsert(queue, i, vwgt[i] - rinfo[i].edegrees[other]);
    }

    ASSERT(CheckNodeBnd(graph, nbnd));
    ASSERT(CheckNodePartitionParams(graph));

    /******************************************************
  * Get into the FM loop
  *******************************************************/
    for(nswaps = 0; nswaps < nvtxs; nswaps++)
    {
        if((higain = rpqGetTop(queue)) == -1)
            break;

        moved[higain] = 1;

        gain       = vwgt[higain] - rinfo[higain].edegrees[other];
        badmaxpwgt = (idx_t)(mult * (pwgts[0] + pwgts[1]));

        /* break if other is now underwight */
        if(pwgts[to] > pwgts[other])
            break;

        /* break if balance is achieved and no +ve or zero gain */
        if(gain < 0 && pwgts[other] < badmaxpwgt)
            break;

        /* skip this vertex if it will violate balance on the other side */
        if(pwgts[to] + vwgt[higain] > badmaxpwgt)
            continue;

        ASSERT(bndptr[higain] != -1);

        pwgts[2] -= gain;

        BNDDelete(nbnd, bndind, bndptr, higain);
        pwgts[to] += vwgt[higain];
        where[higain] = to;

        IFSET(ctrl->dbglvl,
              METIS_DBG_MOVEINFO,
              printf("Moved %6" PRIDX " to %3" PRIDX ", Gain: %3" PRIDX
                     ", \t[%5" PRIDX " %5" PRIDX " %5" PRIDX "]\n",
                     higain,
                     to,
                     vwgt[higain] - rinfo[higain].edegrees[other],
                     pwgts[0],
                     pwgts[1],
                     pwgts[2]));


        /**********************************************************
    * Update the degrees of the affected nodes
    ***********************************************************/
        for(j = xadj[higain]; j < xadj[higain + 1]; j++)
        {
            k = adjncy[j];
            if(where[k] == 2)
            { /* For the in-separator vertices modify their edegree[to] */
                rinfo[k].edegrees[to] += vwgt[higain];
            }
            else if(where[k] == other)
            { /* This vertex is pulled into the separator */
                ASSERTP(bndptr[k] == -1,
                        ("%" PRIDX " %" PRIDX " %" PRIDX "\n", k, bndptr[k], where[k]));
                BNDInsert(nbnd, bndind, bndptr, k);

                where[k] = 2;
                pwgts[other] -= vwgt[k];

                edegrees    = rinfo[k].edegrees;
                edegrees[0] = edegrees[1] = 0;
                for(jj = xadj[k]; jj < xadj[k + 1]; jj++)
                {
                    kk = adjncy[jj];
                    if(where[kk] != 2)
                        edegrees[where[kk]] += vwgt[kk];
                    else
                    {
                        ASSERT(bndptr[kk] != -1);
                        oldgain = vwgt[kk] - rinfo[kk].edegrees[other];
                        rinfo[kk].edegrees[other] -= vwgt[k];

                        if(moved[kk] == -1)
                            rpqUpdate(queue, kk, oldgain + vwgt[k]);
                    }
                }

                /* Insert the new vertex into the priority queue */
                rpqInsert(queue, k, vwgt[k] - edegrees[other]);
            }
        }
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_REFINE,
          printf("\tBalanced sep: %6" PRIDX " at %4" PRIDX ", PWGTS: [%6" PRIDX
                 " %6" PRIDX "], NBND: %6" PRIDX "\n",
                 pwgts[2],
                 nswaps,
                 pwgts[0],
                 pwgts[1],
                 nbnd));

    graph->mincut = pwgts[2];
    graph->nbnd   = nbnd;

    rpqDestroy(queue);

    WCOREPOP;
}
