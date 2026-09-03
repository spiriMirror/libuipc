/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 * This file is split from the port's merged implementation.
 */

#include <metis.h>

/* The merged implementation removes unused source files and GK_MK* macro
 * instantiations while preserving the METIS_PartGraphKway algorithm.
 */

/************************ bucketsort.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * bucketsort.c
 *
 * This file contains code that implement a variety of counting sorting
 * algorithms
 *
 * Started 7/25/97
 * George
 *
 */


/*************************************************************************
* This function uses simple counting sort to return a permutation array
* corresponding to the sorted order. The keys are arsumed to start from
* 0 and they are positive.  This sorting is used during matching.
**************************************************************************/
void BucketSortKeysInc(ctrl_t* ctrl, idx_t n, idx_t max, idx_t* keys, idx_t* tperm, idx_t* perm)
{
    idx_t  i, ii;
    idx_t* counts;

    WCOREPUSH;

    counts = iset(max + 2, 0, iwspacemalloc(ctrl, max + 2));

    for(i = 0; i < n; i++)
        counts[keys[i]]++;
    MAKECSR(i, max + 1, counts);

    for(ii = 0; ii < n; ii++)
    {
        i                       = tperm[ii];
        perm[counts[keys[i]]++] = i;
    }

    WCOREPOP;
}

/************************ coarsen.c ************************/
/*!
\file
\brief Functions for computing matchings during graph coarsening

\date Started 7/23/97
\author George
\author Copyright 1997-2011, Regents of the University of Minnesota
\version\verbatim $Id: coarsen.c 20398 2016-11-22 17:17:12Z karypis $ \endverbatim
*/


#define UNMATCHEDFOR2HOP                                                       \
    0.10 /* The fraction of unmatched vertices that triggers 2-hop */


/*************************************************************************/
/*! This function takes a graph and creates a sequence of coarser graphs.
    It implements the coarsening phase of the multilevel paradigm.
 */
/*************************************************************************/
graph_t* CoarsenGraph(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, eqewgts, level = 0;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->CoarsenTmr));

    /* determine if the weights on the edges are all the same */
    for(eqewgts = 1, i = 1; i < graph->nedges; i++)
    {
        if(graph->adjwgt[0] != graph->adjwgt[i])
        {
            eqewgts = 0;
            break;
        }
    }

    /* set the maximum allowed coarsest vertex weight */
    for(i = 0; i < graph->ncon; i++)
        ctrl->maxvwgt[i] = 1.5 * graph->tvwgt[i] / ctrl->CoarsenTo;

    do
    {
        IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));

        /* allocate memory for cmap, if it has not already been done due to
       multiple cuts */
        if(graph->cmap == NULL)
            graph->cmap = imalloc(graph->nvtxs, "CoarsenGraph: graph->cmap");

        /* determine which matching scheme you will use */
        switch(ctrl->ctype)
        {
            case METIS_CTYPE_RM:
                Match_RM(ctrl, graph);
                break;
            case METIS_CTYPE_SHEM:
                if(eqewgts || graph->nedges == 0)
                    Match_RM(ctrl, graph);
                else
                    Match_SHEM(ctrl, graph);
                break;
            default:
                gk_errexit(SIGERR, "Unknown ctype: %d\n", ctrl->ctype);
        }

        graph_WriteToDisk(ctrl, graph);

        graph   = graph->coarser;
        eqewgts = 0;
        level++;

        ASSERT(CheckGraph(graph, 0, 1));

    } while(graph->nvtxs > ctrl->CoarsenTo
            && graph->nvtxs < COARSEN_FRACTION * graph->finer->nvtxs
            && graph->nedges > graph->nvtxs / 2);

    IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->CoarsenTmr));

    return graph;
}


/*************************************************************************/
/*! This function takes a graph and creates a sequence of nlevels coarser
    graphs, where nlevels is an input parameter.
 */
/*************************************************************************/
graph_t* CoarsenGraphNlevels(ctrl_t* ctrl, graph_t* graph, idx_t nlevels)
{
    idx_t i, eqewgts, level;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->CoarsenTmr));

    /* determine if the weights on the edges are all the same */
    for(eqewgts = 1, i = 1; i < graph->nedges; i++)
    {
        if(graph->adjwgt[0] != graph->adjwgt[i])
        {
            eqewgts = 0;
            break;
        }
    }

    /* set the maximum allowed coarsest vertex weight */
    for(i = 0; i < graph->ncon; i++)
        ctrl->maxvwgt[i] = 1.5 * graph->tvwgt[i] / ctrl->CoarsenTo;

    for(level = 0; level < nlevels; level++)
    {
        IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));

        /* allocate memory for cmap, if it has not already been done due to
       multiple cuts */
        if(graph->cmap == NULL)
            graph->cmap = imalloc(graph->nvtxs, "CoarsenGraph: graph->cmap");

        /* determine which matching scheme you will use */
        switch(ctrl->ctype)
        {
            case METIS_CTYPE_RM:
                Match_RM(ctrl, graph);
                break;
            case METIS_CTYPE_SHEM:
                if(eqewgts || graph->nedges == 0)
                    Match_RM(ctrl, graph);
                else
                    Match_SHEM(ctrl, graph);
                break;
            default:
                gk_errexit(SIGERR, "Unknown ctype: %d\n", ctrl->ctype);
        }

        graph_WriteToDisk(ctrl, graph);

        graph   = graph->coarser;
        eqewgts = 0;

        ASSERT(CheckGraph(graph, 0, 1));

        if(graph->nvtxs < ctrl->CoarsenTo
           || graph->nvtxs > COARSEN_FRACTION * graph->finer->nvtxs
           || graph->nedges < graph->nvtxs / 2)
            break;
    }

    IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->CoarsenTmr));

    return graph;
}


/*************************************************************************/
/*! This function finds a matching by randomly selecting one of the
    unmatched adjacent vertices.
 */
/**************************************************************************/
idx_t Match_RM(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, pi, ii, j, jj, jjinc, k, nvtxs, ncon, cnvtxs, maxidx,
        last_unmatched, avgdegree, bnum;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *maxvwgt;
    idx_t *match, *cmap, *degrees, *perm, *tperm;
    size_t nunmatched = 0;

    WCOREPUSH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    cmap   = graph->cmap;

    maxvwgt = ctrl->maxvwgt;

    match   = iset(nvtxs, UNMATCHED, iwspacemalloc(ctrl, nvtxs));
    perm    = iwspacemalloc(ctrl, nvtxs);
    tperm   = iwspacemalloc(ctrl, nvtxs);
    degrees = iwspacemalloc(ctrl, nvtxs);

    /* Determine a "random" traversal order that is biased towards
     low-degree vertices */
    irandArrayPermute(nvtxs, tperm, nvtxs / 8, 1);

    avgdegree = 4.0 * (xadj[nvtxs] / nvtxs);
    for(i = 0; i < nvtxs; i++)
    {
        bnum       = sqrt(1 + xadj[i + 1] - xadj[i]);
        degrees[i] = (bnum > avgdegree ? avgdegree : bnum);
    }
    BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees, tperm, perm);


    /* Traverse the vertices and compute the matching */
    for(cnvtxs = 0, last_unmatched = 0, pi = 0; pi < nvtxs; pi++)
    {
        i = perm[pi];

        if(match[i] == UNMATCHED)
        { /* Unmatched */
            maxidx = i;

            if((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt + i * ncon, maxvwgt)))
            {
                /* Deal with island vertices. Find a non-island and match it with.
           The matching ignores ctrl->maxvwgt requirements */
                if(xadj[i] == xadj[i + 1])
                {
                    last_unmatched = gk_max(pi, last_unmatched) + 1;
                    for(; last_unmatched < nvtxs; last_unmatched++)
                    {
                        j = perm[last_unmatched];
                        if(match[j] == UNMATCHED)
                        {
                            maxidx = j;
                            break;
                        }
                    }
                }
                else
                {
                    /* Find a random matching, subject to maxvwgt constraints */
                    if(ncon == 1)
                    {
                        /* single constraint version */
                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            k = adjncy[j];
                            if(match[k] == UNMATCHED && vwgt[i] + vwgt[k] <= maxvwgt[0])
                            {
                                maxidx = k;
                                break;
                            }
                        }

                        /* If it did not match, record for a 2-hop matching. */
                        if(maxidx == i && 2 * vwgt[i] < maxvwgt[0])
                        {
                            nunmatched++;
                            maxidx = UNMATCHED;
                        }
                    }
                    else
                    {
                        /* multi-constraint version */
                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            k = adjncy[j];
                            if(match[k] == UNMATCHED
                               && ivecaxpylez(ncon, 1, vwgt + i * ncon, vwgt + k * ncon, maxvwgt))
                            {
                                maxidx = k;
                                break;
                            }
                        }

                        /* If it did not match, record for a 2-hop matching. */
                        if(maxidx == i
                           && ivecaxpylez(ncon, 2, vwgt + i * ncon, vwgt + i * ncon, maxvwgt))
                        {
                            nunmatched++;
                            maxidx = UNMATCHED;
                        }
                    }
                }
            }

            if(maxidx != UNMATCHED)
            {
                cmap[i] = cmap[maxidx] = cnvtxs++;
                match[i]               = maxidx;
                match[maxidx]          = i;
            }
        }
    }

    //printf("nunmatched: %zu\n", nunmatched);

    /* see if a 2-hop matching is required/allowed */
    if(!ctrl->no2hop && nunmatched > UNMATCHEDFOR2HOP * nvtxs)
        cnvtxs = Match_2Hop(ctrl, graph, perm, match, cnvtxs, nunmatched);


    /* match the final unmatched vertices with themselves and reorder the vertices
     of the coarse graph for memory-friendly contraction */
    for(cnvtxs = 0, i = 0; i < nvtxs; i++)
    {
        if(match[i] == UNMATCHED)
        {
            match[i] = i;
            cmap[i]  = cnvtxs++;
        }
        else
        {
            if(i <= match[i])
                cmap[i] = cmap[match[i]] = cnvtxs++;
        }
    }

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

    CreateCoarseGraph(ctrl, graph, cnvtxs, match);

    WCOREPOP;

    return cnvtxs;
}


/**************************************************************************/
/*! This function finds a matching using the HEM heuristic. The vertices
    are visited based on increasing degree to ensure that all vertices are
    given a chance to match with something.
 */
/**************************************************************************/
idx_t Match_SHEM(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, pi, ii, j, jj, jjinc, k, nvtxs, ncon, cnvtxs, maxidx, maxwgt,
        last_unmatched, avgdegree, bnum;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *maxvwgt;
    idx_t *match, *cmap, *degrees, *perm, *tperm;
    size_t nunmatched = 0;

    WCOREPUSH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    cmap   = graph->cmap;

    maxvwgt = ctrl->maxvwgt;

    match   = iset(nvtxs, UNMATCHED, iwspacemalloc(ctrl, nvtxs));
    perm    = iwspacemalloc(ctrl, nvtxs);
    tperm   = iwspacemalloc(ctrl, nvtxs);
    degrees = iwspacemalloc(ctrl, nvtxs);

    /* Determine a "random" traversal order that is biased towards low-degree vertices */
    irandArrayPermute(nvtxs, tperm, nvtxs / 8, 1);

    avgdegree = 4.0 * (xadj[nvtxs] / nvtxs);
    for(i = 0; i < nvtxs; i++)
    {
        bnum       = sqrt(1 + xadj[i + 1] - xadj[i]);
        degrees[i] = (bnum > avgdegree ? avgdegree : bnum);
    }
    BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees, tperm, perm);


    /* Traverse the vertices and compute the matching */
    for(cnvtxs = 0, last_unmatched = 0, pi = 0; pi < nvtxs; pi++)
    {
        i = perm[pi];

        if(match[i] == UNMATCHED)
        { /* Unmatched */
            maxidx = i;
            maxwgt = -1;

            if((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt + i * ncon, maxvwgt)))
            {
                /* Deal with island vertices. Find a non-island and match it with.
           The matching ignores ctrl->maxvwgt requirements */
                if(xadj[i] == xadj[i + 1])
                {
                    last_unmatched = gk_max(pi, last_unmatched) + 1;
                    for(; last_unmatched < nvtxs; last_unmatched++)
                    {
                        j = perm[last_unmatched];
                        if(match[j] == UNMATCHED)
                        {
                            maxidx = j;
                            break;
                        }
                    }
                }
                else
                {
                    /* Find a heavy-edge matching, subject to maxvwgt constraints */
                    if(ncon == 1)
                    {
                        /* single constraint version */
                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            k = adjncy[j];
                            if(match[k] == UNMATCHED && maxwgt < adjwgt[j]
                               && vwgt[i] + vwgt[k] <= maxvwgt[0])
                            {
                                maxidx = k;
                                maxwgt = adjwgt[j];
                            }
                        }

                        /* If it did not match, record for a 2-hop matching. */
                        if(maxidx == i && 2 * vwgt[i] < maxvwgt[0])
                        {
                            nunmatched++;
                            maxidx = UNMATCHED;
                        }
                    }
                    else
                    {
                        /* multi-constraint version */
                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            k = adjncy[j];
                            if(match[k] == UNMATCHED
                               && ivecaxpylez(ncon, 1, vwgt + i * ncon, vwgt + k * ncon, maxvwgt)
                               && (maxwgt < adjwgt[j]
                                   || (maxwgt == adjwgt[j]
                                       && BetterVBalance(ncon,
                                                         graph->invtvwgt,
                                                         vwgt + i * ncon,
                                                         vwgt + maxidx * ncon,
                                                         vwgt + k * ncon))))
                            {
                                maxidx = k;
                                maxwgt = adjwgt[j];
                            }
                        }

                        /* If it did not match, record for a 2-hop matching. */
                        if(maxidx == i
                           && ivecaxpylez(ncon, 2, vwgt + i * ncon, vwgt + i * ncon, maxvwgt))
                        {
                            nunmatched++;
                            maxidx = UNMATCHED;
                        }
                    }
                }
            }

            if(maxidx != UNMATCHED)
            {
                cmap[i] = cmap[maxidx] = cnvtxs++;
                match[i]               = maxidx;
                match[maxidx]          = i;
            }
        }
    }

    //printf("nunmatched: %zu\n", nunmatched);

    /* see if a 2-hop matching is required/allowed */
    if(!ctrl->no2hop && nunmatched > UNMATCHEDFOR2HOP * nvtxs)
        cnvtxs = Match_2Hop(ctrl, graph, perm, match, cnvtxs, nunmatched);


    /* match the final unmatched vertices with themselves and reorder the vertices
     of the coarse graph for memory-friendly contraction */
    for(cnvtxs = 0, i = 0; i < nvtxs; i++)
    {
        if(match[i] == UNMATCHED)
        {
            match[i] = i;
            cmap[i]  = cnvtxs++;
        }
        else
        {
            if(i <= match[i])
                cmap[i] = cmap[match[i]] = cnvtxs++;
        }
    }

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

    CreateCoarseGraph(ctrl, graph, cnvtxs, match);

    WCOREPOP;

    return cnvtxs;
}


/*************************************************************************/
/*! This function matches the unmatched vertices using a 2-hop matching
    that involves vertices that are two hops away from each other. */
/**************************************************************************/
idx_t Match_2Hop(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match, idx_t cnvtxs, size_t nunmatched)
{

    cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, 2);
    cnvtxs = Match_2HopAll(ctrl, graph, perm, match, cnvtxs, &nunmatched, 64);
    if(nunmatched > 1.5 * UNMATCHEDFOR2HOP * graph->nvtxs)
        cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, 3);
    if(nunmatched > 2.0 * UNMATCHEDFOR2HOP * graph->nvtxs)
        cnvtxs =
            Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, graph->nvtxs);

    return cnvtxs;
}


/*************************************************************************/
/*! This function matches the unmatched vertices whose degree is less than
    maxdegree using a 2-hop matching that involves vertices that are two
    hops away from each other.
    The requirement of the 2-hop matching is a simple non-empty overlap
    between the adjacency lists of the vertices. */
/**************************************************************************/
idx_t Match_2HopAny(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match, idx_t cnvtxs, size_t* r_nunmatched, size_t maxdegree)
{
    idx_t  i, pi, ii, j, jj, k, nvtxs;
    idx_t *xadj, *adjncy, *colptr, *rowind;
    idx_t* cmap;
    size_t nunmatched;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux3Tmr));

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    cmap   = graph->cmap;

    nunmatched = *r_nunmatched;

    /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("IN: nunmatched: %zu\t", nunmatched)); */

    /* create the inverted index */
    WCOREPUSH;
    colptr = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs + 1));
    for(i = 0; i < nvtxs; i++)
    {
        if(match[i] == UNMATCHED && xadj[i + 1] - xadj[i] < maxdegree)
        {
            for(j = xadj[i]; j < xadj[i + 1]; j++)
                colptr[adjncy[j]]++;
        }
    }
    MAKECSR(i, nvtxs, colptr);

    rowind = iwspacemalloc(ctrl, colptr[nvtxs]);
    for(pi = 0; pi < nvtxs; pi++)
    {
        i = perm[pi];
        if(match[i] == UNMATCHED && xadj[i + 1] - xadj[i] < maxdegree)
        {
            for(j = xadj[i]; j < xadj[i + 1]; j++)
                rowind[colptr[adjncy[j]]++] = i;
        }
    }
    SHIFTCSR(i, nvtxs, colptr);

    /* compute matchings by going down the inverted index */
    for(pi = 0; pi < nvtxs; pi++)
    {
        i = perm[pi];
        if(colptr[i + 1] - colptr[i] < 2)
            continue;

        for(jj = colptr[i + 1], j = colptr[i]; j < jj; j++)
        {
            if(match[rowind[j]] == UNMATCHED)
            {
                for(jj--; jj > j; jj--)
                {
                    if(match[rowind[jj]] == UNMATCHED)
                    {
                        cmap[rowind[j]] = cmap[rowind[jj]] = cnvtxs++;
                        match[rowind[j]]                   = rowind[jj];
                        match[rowind[jj]]                  = rowind[j];
                        nunmatched -= 2;
                        break;
                    }
                }
            }
        }
    }
    WCOREPOP;

    /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("OUT: nunmatched: %zu\n", nunmatched)); */

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux3Tmr));

    *r_nunmatched = nunmatched;
    return cnvtxs;
}


/*************************************************************************/
/*! This function matches the unmatched vertices whose degree is less than
    maxdegree using a 2-hop matching that involves vertices that are two
    hops away from each other.
    The requirement of the 2-hop matching is that of identical adjacency
    lists.
 */
/**************************************************************************/
idx_t Match_2HopAll(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match, idx_t cnvtxs, size_t* r_nunmatched, size_t maxdegree)
{
    idx_t  i, pi, pk, ii, j, jj, k, nvtxs, mask, idegree;
    idx_t *xadj, *adjncy;
    idx_t *cmap, *mark;
    ikv_t* keys;
    size_t nunmatched, ncand;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux3Tmr));

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    cmap   = graph->cmap;

    nunmatched = *r_nunmatched;
    mask       = IDX_MAX / maxdegree;

    /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("IN: nunmatched: %zu\t", nunmatched)); */

    WCOREPUSH;

    /* collapse vertices with identical adjacency lists */
    keys = ikvwspacemalloc(ctrl, nunmatched);
    for(ncand = 0, pi = 0; pi < nvtxs; pi++)
    {
        i       = perm[pi];
        idegree = xadj[i + 1] - xadj[i];
        if(match[i] == UNMATCHED && idegree > 1 && idegree < maxdegree)
        {
            for(k = 0, j = xadj[i]; j < xadj[i + 1]; j++)
                k += adjncy[j] % mask;
            keys[ncand].val = i;
            keys[ncand].key = (k % mask) * maxdegree + idegree;
            ncand++;
        }
    }
    ikvsorti(ncand, keys);

    mark = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    for(pi = 0; pi < ncand; pi++)
    {
        i = keys[pi].val;
        if(match[i] != UNMATCHED)
            continue;

        for(j = xadj[i]; j < xadj[i + 1]; j++)
            mark[adjncy[j]] = i;

        for(pk = pi + 1; pk < ncand; pk++)
        {
            k = keys[pk].val;
            if(match[k] != UNMATCHED)
                continue;

            if(keys[pi].key != keys[pk].key)
                break;
            if(xadj[i + 1] - xadj[i] != xadj[k + 1] - xadj[k])
                break;

            for(jj = xadj[k]; jj < xadj[k + 1]; jj++)
            {
                if(mark[adjncy[jj]] != i)
                    break;
            }
            if(jj == xadj[k + 1])
            {
                cmap[i] = cmap[k] = cnvtxs++;
                match[i]          = k;
                match[k]          = i;
                nunmatched -= 2;
                break;
            }
        }
    }
    WCOREPOP;

    /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("OUT: ncand: %zu, nunmatched: %zu\n", ncand, nunmatched)); */

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux3Tmr));

    *r_nunmatched = nunmatched;
    return cnvtxs;
}


/*************************************************************************/
/*! This function finds a matching by selecting an adjacent vertex based
    on the Jaccard coefficient of the adjaceny lists.
 */
/**************************************************************************/
idx_t Match_JC(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, pi, ii, iii, j, jj, jjj, jjinc, k, nvtxs, ncon, cnvtxs, maxidx,
        last_unmatched, avgdegree, bnum;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *maxvwgt;
    idx_t *match, *cmap, *degrees, *perm, *tperm, *vec, *marker;
    idx_t  mytwgt, xtwgt, ctwgt;
    real_t bscore, score;

    WCOREPUSH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    cmap   = graph->cmap;

    maxvwgt = ctrl->maxvwgt;

    match   = iset(nvtxs, UNMATCHED, iwspacemalloc(ctrl, nvtxs));
    perm    = iwspacemalloc(ctrl, nvtxs);
    tperm   = iwspacemalloc(ctrl, nvtxs);
    degrees = iwspacemalloc(ctrl, nvtxs);

    irandArrayPermute(nvtxs, tperm, nvtxs / 8, 1);

    avgdegree = 4.0 * (xadj[nvtxs] / nvtxs);
    for(i = 0; i < nvtxs; i++)
    {
        bnum       = sqrt(1 + xadj[i + 1] - xadj[i]);
        degrees[i] = (bnum > avgdegree ? avgdegree : bnum);
    }
    BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees, tperm, perm);

    /* point to the wspace vectors that are not needed any more */
    vec    = tperm;
    marker = degrees;
    iset(nvtxs, -1, vec);
    iset(nvtxs, -1, marker);

    for(cnvtxs = 0, last_unmatched = 0, pi = 0; pi < nvtxs; pi++)
    {
        i = perm[pi];

        if(match[i] == UNMATCHED)
        { /* Unmatched */
            maxidx = i;

            if((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt + i * ncon, maxvwgt)))
            {
                /* Deal with island vertices. Find a non-island and match it with.
           The matching ignores ctrl->maxvwgt requirements */
                if(xadj[i] == xadj[i + 1])
                {
                    last_unmatched = gk_max(pi, last_unmatched) + 1;
                    for(; last_unmatched < nvtxs; last_unmatched++)
                    {
                        j = perm[last_unmatched];
                        if(match[j] == UNMATCHED)
                        {
                            maxidx = j;
                            break;
                        }
                    }
                }
                else
                {
                    if(ncon == 1)
                    {
                        /* Find a max JC pair, subject to maxvwgt constraints */
                        if(xadj[i + 1] - xadj[i] < avgdegree)
                        {
                            marker[i] = i;
                            bscore    = 0.0;
                            mytwgt    = 0;
                            for(j = xadj[i]; j < xadj[i + 1]; j++)
                            {
                                mytwgt += 1;         //adjwgt[j];
                                vec[adjncy[j]] = 1;  //adjwgt[j];
                            }

                            /* single constraint pairing */
#ifdef XXX
                            for(j = xadj[i]; j < xadj[i + 1]; j++)
                            {
                                ii = adjncy[j];
                                if(marker[ii] == i || match[ii] != UNMATCHED
                                   || vwgt[i] + vwgt[ii] > maxvwgt[0])
                                    continue;

                                ctwgt = xtwgt = 0;
                                for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                                {
                                    xtwgt += adjwgt[jj];
                                    if(vec[adjncy[jj]] > 0)
                                        ctwgt += vec[adjncy[jj]] + adjwgt[jj];
                                    else if(adjncy[jj] == i)
                                    {
                                        ctwgt += adjwgt[jj];
                                        xtwgt -= adjwgt[jj];
                                    }
                                }

                                score = 1.0 * ctwgt / (mytwgt + xtwgt - ctwgt);
                                if(score > bscore)
                                {
                                    bscore = score;
                                    maxidx = ii;
                                }
                                marker[ii] = i;
                            }
#endif

                            for(j = xadj[i]; j < xadj[i + 1]; j++)
                            {
                                ii = adjncy[j];
                                for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                                {
                                    iii = adjncy[jj];

                                    if(marker[iii] == i || match[iii] != UNMATCHED
                                       || vwgt[i] + vwgt[iii] > maxvwgt[0])
                                        continue;

                                    ctwgt = xtwgt = 0;
                                    for(jjj = xadj[iii]; jjj < xadj[iii + 1]; jjj++)
                                    {
                                        xtwgt += 1;  //adjwgt[jjj];
                                        if(vec[adjncy[jjj]] > 0)
                                            ctwgt += 2;  //vec[adjncy[jjj]] + adjwgt[jjj];
                                        else if(adjncy[jjj] == i)
                                            ctwgt += 10 * adjwgt[jjj];
                                    }

                                    score = 1.0 * ctwgt / (mytwgt + xtwgt);
                                    //printf("%"  PRIDX  " %"  PRIDX  " %"  PRIDX  " %.4f\n", mytwgt, xtwgt, ctwgt, score);
                                    if(score > bscore)
                                    {
                                        bscore = score;
                                        maxidx = iii;
                                    }
                                    marker[iii] = i;
                                }
                            }

                            /* reset vec array */
                            for(j = xadj[i]; j < xadj[i + 1]; j++)
                                vec[adjncy[j]] = -1;
                        }
                    }
                    else
                    {
                        /* multi-constraint version */
                        for(j = xadj[i]; j < xadj[i + 1]; j++)
                        {
                            k = adjncy[j];
                            if(match[k] == UNMATCHED
                               && ivecaxpylez(ncon, 1, vwgt + i * ncon, vwgt + k * ncon, maxvwgt))
                            {
                                maxidx = k;
                                break;
                            }
                        }
                    }
                }
            }

            if(maxidx != UNMATCHED)
            {
                cmap[i] = cmap[maxidx] = cnvtxs++;
                match[i]               = maxidx;
                match[maxidx]          = i;
            }
        }
    }


    /* match the final unmatched vertices with themselves and reorder the vertices
     of the coarse graph for memory-friendly contraction */
    for(cnvtxs = 0, i = 0; i < nvtxs; i++)
    {
        if(match[i] == UNMATCHED)
        {
            match[i] = i;
            cmap[i]  = cnvtxs++;
        }
        else
        {
            if(i <= match[i])
                cmap[i] = cmap[match[i]] = cnvtxs++;
        }
    }

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

    CreateCoarseGraph(ctrl, graph, cnvtxs, match);

    WCOREPOP;

    return cnvtxs;
}


/*************************************************************************/
/*! This function prints various stats for each graph during coarsening
 */
/*************************************************************************/
void PrintCGraphStats(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i;

    printf("%10" PRIDX " %10" PRIDX " %10" PRIDX " [%" PRIDX "] [",
           graph->nvtxs,
           graph->nedges,
           isum(graph->nedges, graph->adjwgt, 1),
           ctrl->CoarsenTo);

    for(i = 0; i < graph->ncon; i++)
        printf(" %8" PRIDX ":%8" PRIDX, ctrl->maxvwgt[i], graph->tvwgt[i]);
    printf(" ]\n");
}


/*************************************************************************/
/*! This function creates the coarser graph. Depending on the size of the
    candidate adjacency lists it either uses a hash table or an array
    to do duplicate detection.
 */
/*************************************************************************/
void CreateCoarseGraph(ctrl_t* ctrl, graph_t* graph, idx_t cnvtxs, idx_t* match)
{
    idx_t j, jj, k, kk, l, m, istart, iend, nvtxs, nedges, ncon, cnedges, v, u, mask;
    idx_t *  xadj, *vwgt, *vsize, *adjncy, *adjwgt;
    idx_t *  cmap, *htable, *dtable;
    idx_t *  cxadj, *cvwgt, *cvsize, *cadjncy, *cadjwgt;
    graph_t* cgraph;
    int      dovsize, dropedges;
    idx_t    cv, nkeys, droppedewgt;
    idx_t *  keys = NULL, *medianewgts = NULL, *noise = NULL;

    WCOREPUSH;

    dovsize   = (ctrl->objtype == METIS_OBJTYPE_VOL ? 1 : 0);
    dropedges = ctrl->dropedges;

    mask = HTLENGTH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ContractTmr));

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    vsize  = graph->vsize;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    cmap   = graph->cmap;

    /* Setup structures for dropedges */
    if(dropedges)
    {
        for(nkeys = 0, v = 0; v < nvtxs; v++)
            nkeys = gk_max(nkeys, xadj[v + 1] - xadj[v]);
        nkeys = 2 * nkeys + 1;

        keys        = iwspacemalloc(ctrl, nkeys);
        noise       = iwspacemalloc(ctrl, cnvtxs);
        medianewgts = iset(cnvtxs, -1, iwspacemalloc(ctrl, cnvtxs));

        for(v = 0; v < cnvtxs; v++)
            noise[v] = irandInRange(128);
    }

    /* Initialize the coarser graph */
    cgraph  = SetupCoarseGraph(graph, cnvtxs, dovsize);
    cxadj   = cgraph->xadj;
    cvwgt   = cgraph->vwgt;
    cvsize  = cgraph->vsize;
    cadjncy = cgraph->adjncy;
    cadjwgt = cgraph->adjwgt;

    htable = iset(mask + 1, -1, iwspacemalloc(ctrl, mask + 1)); /* hash table */
    dtable = iset(cnvtxs, -1, iwspacemalloc(ctrl, cnvtxs)); /* direct table */

    cxadj[0] = cnvtxs = cnedges = 0;
    for(v = 0; v < nvtxs; v++)
    {
        if((u = match[v]) < v)
            continue;

        ASSERT(cmap[v] == cnvtxs);
        ASSERT(cmap[match[v]] == cnvtxs);

        /* take care of the vertices */
        if(ncon == 1)
            cvwgt[cnvtxs] = vwgt[v];
        else
            icopy(ncon, vwgt + v * ncon, cvwgt + cnvtxs * ncon);

        if(dovsize)
            cvsize[cnvtxs] = vsize[v];

        if(v != u)
        {
            if(ncon == 1)
                cvwgt[cnvtxs] += vwgt[u];
            else
                iaxpy(ncon, 1, vwgt + u * ncon, 1, cvwgt + cnvtxs * ncon, 1);

            if(dovsize)
                cvsize[cnvtxs] += vsize[u];
        }


        /* take care of the edges */
        if((xadj[v + 1] - xadj[v] + xadj[u + 1] - xadj[u]) < (mask >> 2))
        { /* use mask */
            /* put the ID of the contracted node itself at the start, so that it can be
       * removed easily */
            htable[cnvtxs & mask] = 0;
            cadjncy[0]            = cnvtxs;
            nedges                = 1;

            istart = xadj[v];
            iend   = xadj[v + 1];
            for(j = istart; j < iend; j++)
            {
                k = cmap[adjncy[j]];
                for(kk = k & mask; htable[kk] != -1 && cadjncy[htable[kk]] != k;
                    kk = ((kk + 1) & mask))
                    ;
                if((m = htable[kk]) == -1)
                {
                    cadjncy[nedges] = k;
                    cadjwgt[nedges] = adjwgt[j];
                    htable[kk]      = nedges++;
                }
                else
                {
                    cadjwgt[m] += adjwgt[j];
                }
            }

            if(v != u)
            {
                istart = xadj[u];
                iend   = xadj[u + 1];
                for(j = istart; j < iend; j++)
                {
                    k = cmap[adjncy[j]];
                    for(kk = k & mask; htable[kk] != -1 && cadjncy[htable[kk]] != k;
                        kk = ((kk + 1) & mask))
                        ;
                    if((m = htable[kk]) == -1)
                    {
                        cadjncy[nedges] = k;
                        cadjwgt[nedges] = adjwgt[j];
                        htable[kk]      = nedges++;
                    }
                    else
                    {
                        cadjwgt[m] += adjwgt[j];
                    }
                }
            }

            /* reset the htable -- reverse order (LIFO) is critical to prevent cadjncy[-1]
       * indexing due to a remove of an earlier entry */
            for(j = nedges - 1; j >= 0; j--)
            {
                k = cadjncy[j];
                for(kk = k & mask; cadjncy[htable[kk]] != k; kk = ((kk + 1) & mask))
                    ;
                htable[kk] = -1;
            }

            /* remove the contracted vertex from the list */
            cadjncy[0] = cadjncy[--nedges];
            cadjwgt[0] = cadjwgt[nedges];
        }
        else
        {
            nedges = 0;
            istart = xadj[v];
            iend   = xadj[v + 1];
            for(j = istart; j < iend; j++)
            {
                k = cmap[adjncy[j]];
                if((m = dtable[k]) == -1)
                {
                    cadjncy[nedges] = k;
                    cadjwgt[nedges] = adjwgt[j];
                    dtable[k]       = nedges++;
                }
                else
                {
                    cadjwgt[m] += adjwgt[j];
                }
            }

            if(v != u)
            {
                istart = xadj[u];
                iend   = xadj[u + 1];
                for(j = istart; j < iend; j++)
                {
                    k = cmap[adjncy[j]];
                    if((m = dtable[k]) == -1)
                    {
                        cadjncy[nedges] = k;
                        cadjwgt[nedges] = adjwgt[j];
                        dtable[k]       = nedges++;
                    }
                    else
                    {
                        cadjwgt[m] += adjwgt[j];
                    }
                }

                /* Remove the contracted self-loop, when present */
                if((j = dtable[cnvtxs]) != -1)
                {
                    ASSERT(cadjncy[j] == cnvtxs);
                    cadjncy[j]     = cadjncy[--nedges];
                    cadjwgt[j]     = cadjwgt[nedges];
                    dtable[cnvtxs] = -1;
                }
            }

            /* Zero out the dtable */
            for(j = 0; j < nedges; j++)
                dtable[cadjncy[j]] = -1;
        }


        /* Determine the median weight of the incident edges, which will be used
       to keep an edge (u, v) iff wgt(u, v) >= min(medianewgts[u], medianewgts[v]) */
        if(dropedges)
        {
            ASSERTP(nedges < nkeys, ("%" PRIDX ", %" PRIDX "\n", nkeys, nedges));
            medianewgts[cnvtxs] = 8; /* default for island nodes */
            if(nedges > 0)
            {
                for(j = 0; j < nedges; j++)
                    keys[j] = (cadjwgt[j] << 8) + noise[cnvtxs] + noise[cadjncy[j]];
                isortd(nedges, keys);
                medianewgts[cnvtxs] =
                    keys[gk_min(nedges - 1, ((xadj[v + 1] - xadj[v] + xadj[u + 1] - xadj[u]) >> 1))];
            }
        }

        cadjncy += nedges;
        cadjwgt += nedges;
        cnedges += nedges;
        cxadj[++cnvtxs] = cnedges;
    }


    /* compact the adjacency structure of the coarser graph to keep only +ve edges */
    if(dropedges)
    {
        droppedewgt = 0;

        cadjncy = cgraph->adjncy;
        cadjwgt = cgraph->adjwgt;

        cnedges = 0;
        for(u = 0; u < cnvtxs; u++)
        {
            istart = cxadj[u];
            iend   = cxadj[u + 1];
            for(j = istart; j < iend; j++)
            {
                v = cadjncy[j];
                ASSERTP(medianewgts[u] >= 0,
                        ("%" PRIDX " %" PRIDX "\n", u, medianewgts[u]));
                ASSERTP(medianewgts[v] >= 0,
                        ("%" PRIDX " %" PRIDX " %" PRIDX "\n", v, medianewgts[v], cnvtxs));
                if((cadjwgt[j] << 8) + noise[u] + noise[v]
                   >= gk_min(medianewgts[u], medianewgts[v]))
                {
                    cadjncy[cnedges]   = cadjncy[j];
                    cadjwgt[cnedges++] = cadjwgt[j];
                }
                else
                    droppedewgt += cadjwgt[j];
            }
            cxadj[u] = cnedges;
        }
        SHIFTCSR(j, cnvtxs, cxadj);

        cgraph->droppedewgt = droppedewgt;
    }

    cgraph->nedges = cnedges;

    for(j = 0; j < ncon; j++)
    {
        cgraph->tvwgt[j] = isum(cgraph->nvtxs, cgraph->vwgt + j, ncon);
        cgraph->invtvwgt[j] = 1.0 / (cgraph->tvwgt[j] > 0 ? cgraph->tvwgt[j] : 1);
    }

    ReAdjustMemory(ctrl, graph, cgraph);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ContractTmr));

    WCOREPOP;
}


/*************************************************************************/
/*! Setup the various arrays for the coarse graph
 */
/*************************************************************************/
graph_t* SetupCoarseGraph(graph_t* graph, idx_t cnvtxs, int dovsize)
{
    graph_t* cgraph;

    cgraph = CreateGraph();

    cgraph->nvtxs = cnvtxs;
    cgraph->ncon  = graph->ncon;

    cgraph->finer  = graph;
    graph->coarser = cgraph;

    /* Allocate memory for the coarser graph.
     NOTE: The +1 in the adjwgt/adjncy is to allow the optimization of self-loop
           detection by adding ahead of time the self-loop. That optimization
           requires a +1 adjncy/adjwgt array for the limit case where the
           coarser graph is of the same size of the previous graph. */
    cgraph->xadj     = imalloc(cnvtxs + 1, "SetupCoarseGraph: xadj");
    cgraph->adjncy   = imalloc(graph->nedges + 1, "SetupCoarseGraph: adjncy");
    cgraph->adjwgt   = imalloc(graph->nedges + 1, "SetupCoarseGraph: adjwgt");
    cgraph->vwgt     = imalloc(cgraph->ncon * cnvtxs, "SetupCoarseGraph: vwgt");
    cgraph->tvwgt    = imalloc(cgraph->ncon, "SetupCoarseGraph: tvwgt");
    cgraph->invtvwgt = rmalloc(cgraph->ncon, "SetupCoarseGraph: invtvwgt");

    if(dovsize)
        cgraph->vsize = imalloc(cnvtxs, "SetupCoarseGraph: vsize");

    return cgraph;
}


/*************************************************************************/
/*! This function re-adjusts the amount of memory that was allocated if
    it will lead to significant savings
 */
/*************************************************************************/
void ReAdjustMemory(ctrl_t* ctrl, graph_t* graph, graph_t* cgraph)
{
    if(cgraph->nedges > 10000 && cgraph->nedges < 0.9 * graph->nedges)
    {
        cgraph->adjncy = irealloc(cgraph->adjncy, cgraph->nedges, "ReAdjustMemory: adjncy");
        cgraph->adjwgt = irealloc(cgraph->adjwgt, cgraph->nedges, "ReAdjustMemory: adjwgt");
    }
}
/************************ contig.c ************************/
/*!
\file
\brief Functions that deal with eliminating disconnected partitions

\date Started 7/15/98
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version $Id: contig.c 10513 2011-07-07 22:06:03Z karypis $
*/


/*************************************************************************/
/*! This function finds the connected components induced by the
    partitioning vector.

    \param graph is the graph structure
    \param where is the partitioning vector. If this is NULL, then the
           entire graph is treated to belong into a single partition.
    \param cptr is the ptr structure of the CSR representation of the
           components. The length of this vector must be graph->nvtxs+1.
    \param cind is the indices structure of the CSR representation of
           the components. The length of this vector must be graph->nvtxs.

    \returns the number of components that it found.

    \note The cptr and cind parameters can be NULL, in which case only the
          number of connected components is returned.
*/
/*************************************************************************/
idx_t FindPartitionInducedComponents(graph_t* graph, idx_t* where, idx_t* cptr, idx_t* cind)
{
    idx_t  i, ii, j, jj, k, me = 0, nvtxs, first, last, nleft, ncmps;
    idx_t *xadj, *adjncy;
    idx_t *touched, *perm, *todo;
    idx_t  mustfree_ccsr = 0, mustfree_where = 0;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;

    /* Deal with NULL supplied cptr/cind vectors */
    if(cptr == NULL)
    {
        cptr = imalloc(nvtxs + 1, "FindPartitionInducedComponents: cptr");
        cind = imalloc(nvtxs, "FindPartitionInducedComponents: cind");
        mustfree_ccsr = 1;
    }

    /* Deal with NULL supplied where vector */
    if(where == NULL)
    {
        where = ismalloc(nvtxs, 0, "FindPartitionInducedComponents: where");
        mustfree_where = 1;
    }

    /* Allocate memory required for the BFS traversal */
    perm = iincset(nvtxs, 0, imalloc(nvtxs, "FindPartitionInducedComponents: perm"));
    todo = iincset(nvtxs, 0, imalloc(nvtxs, "FindPartitionInducedComponents: todo"));
    touched = ismalloc(nvtxs, 0, "FindPartitionInducedComponents: touched");


    /* Find the connected componends induced by the partition */
    ncmps = -1;
    first = last = 0;
    nleft        = nvtxs;
    while(nleft > 0)
    {
        if(first == last)
        { /* Find another starting vertex */
            cptr[++ncmps] = first;
            ASSERT(touched[todo[0]] == 0);
            i            = todo[0];
            cind[last++] = i;
            touched[i]   = 1;
            me           = where[i];
        }

        i = cind[first++];
        k = perm[i];
        j = todo[k] = todo[--nleft];
        perm[j]     = k;

        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];
            if(where[k] == me && !touched[k])
            {
                cind[last++] = k;
                touched[k]   = 1;
            }
        }
    }
    cptr[++ncmps] = first;

    if(mustfree_ccsr)
        gk_free((void**)&cptr, &cind, LTERM);
    if(mustfree_where)
        gk_free((void**)&where, LTERM);

    gk_free((void**)&perm, &todo, &touched, LTERM);

    return ncmps;
}


/*************************************************************************/
/*! This function computes a permutation of the vertices based on a
    breadth-first-traversal. It can be used for re-ordering the graph
    to reduce its bandwidth for better cache locality.

    \param ctrl is the control structure
    \param graph is the graph structure
    \param perm is the array that upon completion, perm[i] will store
           the ID of the vertex that corresponds to the ith vertex in the
           re-ordered graph.
*/
/*************************************************************************/
void ComputeBFSOrdering(ctrl_t* ctrl, graph_t* graph, idx_t* bfsperm)
{
    idx_t  i, j, k, nvtxs, first, last;
    idx_t *xadj, *adjncy, *perm;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;

    /* Allocate memory required for the BFS traversal */
    perm = iincset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));

    iincset(nvtxs, 0, bfsperm); /* this array will also store the vertices
                                  still to be processed */

    /* Find the connected componends induced by the partition */
    first = last = 0;
    while(first < nvtxs)
    {
        if(first == last)
        { /* Find another starting vertex */
            k = bfsperm[last];
            ASSERT(perm[k] != -1);
            perm[k] = -1; /* mark node as being visited */
            last++;
        }

        i = bfsperm[first++];
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];
            /* if a node has been already been visited, its perm[] will be -1 */
            if(perm[k] != -1)
            {
                /* perm[k] is the location within bfsperm of where k resides;
           put in that location bfsperm[last] that we are about to
           overwrite and update perm[bfsperm[last]] to reflect that. */
                bfsperm[perm[k]]    = bfsperm[last];
                perm[bfsperm[last]] = perm[k];

                bfsperm[last++] = k;  /* put node at the end of the "queue" */
                perm[k]         = -1; /* mark node as being visited */
            }
        }
    }

    WCOREPOP;
}


/*************************************************************************/
/*! This function checks whether a graph is contiguous or not.
 */
/**************************************************************************/
idx_t IsConnected(graph_t* graph, idx_t report)
{
    idx_t ncmps;

    ncmps = FindPartitionInducedComponents(graph, NULL, NULL, NULL);

    if(ncmps != 1 && report)
        printf("The graph is not connected. It has %" PRIDX " connected components.\n", ncmps);

    return (ncmps == 1);
}


/*************************************************************************/
/*! This function checks whether or not partition pid is contiguous
  */
/*************************************************************************/
idx_t IsConnectedSubdomain(ctrl_t* ctrl, graph_t* graph, idx_t pid, idx_t report)
{
    idx_t  i, j, k, nvtxs, first, last, nleft, ncmps, wgt;
    idx_t *xadj, *adjncy, *where, *touched, *queue;
    idx_t* cptr;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    where  = graph->where;

    touched = ismalloc(nvtxs, 0, "IsConnected: touched");
    queue   = imalloc(nvtxs, "IsConnected: queue");
    cptr    = imalloc(nvtxs + 1, "IsConnected: cptr");

    nleft = 0;
    for(i = 0; i < nvtxs; i++)
    {
        if(where[i] == pid)
            nleft++;
    }

    for(i = 0; i < nvtxs; i++)
    {
        if(where[i] == pid)
            break;
    }

    touched[i] = 1;
    queue[0]   = i;
    first      = 0;
    last       = 1;

    cptr[0] = 0; /* This actually points to queue */
    ncmps   = 0;
    while(first != nleft)
    {
        if(first == last)
        { /* Find another starting vertex */
            cptr[++ncmps] = first;
            for(i = 0; i < nvtxs; i++)
            {
                if(where[i] == pid && !touched[i])
                    break;
            }
            queue[last++] = i;
            touched[i]    = 1;
        }

        i = queue[first++];
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];
            if(where[k] == pid && !touched[k])
            {
                queue[last++] = k;
                touched[k]    = 1;
            }
        }
    }
    cptr[++ncmps] = first;

    if(ncmps > 1 && report)
    {
        printf("The graph has %" PRIDX " connected components in partition %" PRIDX ":\t",
               ncmps,
               pid);
        for(i = 0; i < ncmps; i++)
        {
            wgt = 0;
            for(j = cptr[i]; j < cptr[i + 1]; j++)
                wgt += graph->vwgt[queue[j]];
            printf("[%5" PRIDX " %5" PRIDX "] ", cptr[i + 1] - cptr[i], wgt);
            /*
      if (cptr[i+1]-cptr[i] == 1)
        printf("[%"  PRIDX  " %"  PRIDX  "] ", queue[cptr[i]], xadj[queue[cptr[i]]+1]-xadj[queue[cptr[i]]]);
      */
        }
        printf("\n");
    }

    gk_free((void**)&touched, &queue, &cptr, LTERM);

    return (ncmps == 1 ? 1 : 0);
}


/*************************************************************************/
/*! This function identifies the number of connected components in a graph
    that result after removing the vertices that belong to the vertex
    separator (i.e., graph->where[i] == 2).
    The connected component memberships are returned in the CSR-style
    pair of arrays cptr, cind.
*/
/**************************************************************************/
idx_t FindSepInducedComponents(ctrl_t* ctrl, graph_t* graph, idx_t* cptr, idx_t* cind)
{
    idx_t  i, j, k, nvtxs, first, last, nleft, ncmps, wgt;
    idx_t *xadj, *adjncy, *where, *touched, *queue;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    where  = graph->where;

    touched = ismalloc(nvtxs, 0, "IsConnected: queue");

    for(i = 0; i < graph->nbnd; i++)
        touched[graph->bndind[i]] = 1;

    queue = cind;

    nleft = 0;
    for(i = 0; i < nvtxs; i++)
    {
        if(where[i] != 2)
            nleft++;
    }

    for(i = 0; i < nvtxs; i++)
    {
        if(where[i] != 2)
            break;
    }

    touched[i] = 1;
    queue[0]   = i;
    first      = 0;
    last       = 1;
    cptr[0]    = 0; /* This actually points to queue */
    ncmps      = 0;

    while(first != nleft)
    {
        if(first == last)
        { /* Find another starting vertex */
            cptr[++ncmps] = first;
            for(i = 0; i < nvtxs; i++)
            {
                if(!touched[i])
                    break;
            }
            queue[last++] = i;
            touched[i]    = 1;
        }

        i = queue[first++];
        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];
            if(!touched[k])
            {
                queue[last++] = k;
                touched[k]    = 1;
            }
        }
    }
    cptr[++ncmps] = first;

    gk_free((void**)&touched, LTERM);

    return ncmps;
}


/*************************************************************************/
/*! This function finds all the connected components induced by the
    partitioning vector in graph->where and tries to push them around to
    remove some of them. */
/*************************************************************************/
void EliminateComponents(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, ii, j, jj, k, me, nparts, nvtxs, ncon, ncmps, other, ncand, target;
    idx_t * xadj, *adjncy, *vwgt, *adjwgt, *where, *pwgts;
    idx_t * cptr, *cind, *cpvec, *pcptr, *pcind, *cwhere;
    idx_t   cid, bestcid, *cwgt, *bestcwgt;
    idx_t   ntodo, oldntodo, *todo;
    rkv_t*  cand;
    real_t* tpwgts;
    idx_t *vmarker = NULL, *pmarker = NULL, *modind = NULL; /* volume specific work arrays */

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;
    vwgt   = graph->vwgt;
    adjwgt = (ctrl->objtype == METIS_OBJTYPE_VOL ? NULL : graph->adjwgt);

    where = graph->where;
    pwgts = graph->pwgts;

    nparts = ctrl->nparts;
    tpwgts = ctrl->tpwgts;

    cptr = iwspacemalloc(ctrl, nvtxs + 1);
    cind = iwspacemalloc(ctrl, nvtxs);

    ncmps = FindPartitionInducedComponents(graph, where, cptr, cind);

    IFSET(ctrl->dbglvl,
          METIS_DBG_CONTIGINFO,
          printf("I found %" PRIDX " components, for this %" PRIDX "-way partition\n", ncmps, nparts));

    /* There are more components than partitions */
    if(ncmps > nparts)
    {
        cwgt     = iwspacemalloc(ctrl, ncon);
        bestcwgt = iwspacemalloc(ctrl, ncon);
        cpvec    = iwspacemalloc(ctrl, nparts);
        pcptr    = iset(nparts + 1, 0, iwspacemalloc(ctrl, nparts + 1));
        pcind    = iwspacemalloc(ctrl, ncmps);
        cwhere   = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
        todo     = iwspacemalloc(ctrl, ncmps);
        cand     = (rkv_t*)wspacemalloc(ctrl, nparts * sizeof(rkv_t));

        if(ctrl->objtype == METIS_OBJTYPE_VOL)
        {
            /* Vol-refinement specific working arrays */
            modind  = iwspacemalloc(ctrl, nvtxs);
            vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
            pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));
        }


        /* Get a CSR representation of the components-2-partitions mapping */
        for(i = 0; i < ncmps; i++)
            pcptr[where[cind[cptr[i]]]]++;
        MAKECSR(i, nparts, pcptr);
        for(i = 0; i < ncmps; i++)
            pcind[pcptr[where[cind[cptr[i]]]]++] = i;
        SHIFTCSR(i, nparts, pcptr);

        /* Assign the heaviest component of each partition to its original partition */
        for(ntodo = 0, i = 0; i < nparts; i++)
        {
            if(pcptr[i + 1] - pcptr[i] == 1)
                bestcid = pcind[pcptr[i]];
            else
            {
                for(bestcid = -1, j = pcptr[i]; j < pcptr[i + 1]; j++)
                {
                    cid = pcind[j];
                    iset(ncon, 0, cwgt);
                    for(ii = cptr[cid]; ii < cptr[cid + 1]; ii++)
                        iaxpy(ncon, 1, vwgt + cind[ii] * ncon, 1, cwgt, 1);
                    if(bestcid == -1 || isum(ncon, bestcwgt, 1) < isum(ncon, cwgt, 1))
                    {
                        bestcid = cid;
                        icopy(ncon, cwgt, bestcwgt);
                    }
                }
                /* Keep track of those that need to be dealt with */
                for(j = pcptr[i]; j < pcptr[i + 1]; j++)
                {
                    if(pcind[j] != bestcid)
                        todo[ntodo++] = pcind[j];
                }
            }

            for(j = cptr[bestcid]; j < cptr[bestcid + 1]; j++)
            {
                ASSERT(where[cind[j]] == i);
                cwhere[cind[j]] = i;
            }
        }


        while(ntodo > 0)
        {
            oldntodo = ntodo;
            for(i = 0; i < ntodo; i++)
            {
                cid = todo[i];
                me = where[cind[cptr[cid]]]; /* Get the domain of this component */

                /* Determine the weight of the block to be moved */
                iset(ncon, 0, cwgt);
                for(j = cptr[cid]; j < cptr[cid + 1]; j++)
                    iaxpy(ncon, 1, vwgt + cind[j] * ncon, 1, cwgt, 1);

                IFSET(ctrl->dbglvl,
                      METIS_DBG_CONTIGINFO,
                      printf("Trying to move %" PRIDX " [%" PRIDX "] from %" PRIDX "\n",
                             cid,
                             isum(ncon, cwgt, 1),
                             me));

                /* Determine the connectivity */
                iset(nparts, 0, cpvec);
                for(j = cptr[cid]; j < cptr[cid + 1]; j++)
                {
                    ii = cind[j];
                    for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                        if(cwhere[adjncy[jj]] != -1)
                            cpvec[cwhere[adjncy[jj]]] += (adjwgt ? adjwgt[jj] : 1);
                }

                /* Put the neighbors into a cand[] array for sorting */
                for(ncand = 0, j = 0; j < nparts; j++)
                {
                    if(cpvec[j] > 0)
                    {
                        cand[ncand].key   = cpvec[j];
                        cand[ncand++].val = j;
                    }
                }
                if(ncand == 0)
                    continue;

                rkvsortd(ncand, cand);

                /* Limit the moves to only the top candidates, which are defined as
           those with connectivity at least 50% of the best.
           This applies only when ncon=1, as for multi-constraint, balancing
           will be hard. */
                if(ncon == 1)
                {
                    for(j = 1; j < ncand; j++)
                    {
                        if(cand[j].key < .5 * cand[0].key)
                            break;
                    }
                    ncand = j;
                }

                /* Now among those, select the one with the best balance */
                target = cand[0].val;
                for(j = 1; j < ncand; j++)
                {
                    if(BetterBalanceKWay(ncon,
                                         cwgt,
                                         ctrl->ubfactors,
                                         1,
                                         pwgts + target * ncon,
                                         ctrl->pijbm + target * ncon,
                                         1,
                                         pwgts + cand[j].val * ncon,
                                         ctrl->pijbm + cand[j].val * ncon))
                        target = cand[j].val;
                }

                IFSET(ctrl->dbglvl,
                      METIS_DBG_CONTIGINFO,
                      printf("\tMoving it to %" PRIDX " [%" PRIDX "] [%" PRIDX "]\n",
                             target,
                             cpvec[target],
                             ncand));

                /* Note that as a result of a previous movement, a connected component may
           now will like to stay to its original partition */
                if(target != me)
                {
                    switch(ctrl->objtype)
                    {
                        case METIS_OBJTYPE_CUT:
                            MoveGroupContigForCut(ctrl, graph, target, cid, cptr, cind);
                            break;

                        case METIS_OBJTYPE_VOL:
                            MoveGroupContigForVol(
                                ctrl, graph, target, cid, cptr, cind, vmarker, pmarker, modind);
                            break;

                        default:
                            gk_errexit(SIGERR, "Unknown objtype %d\n", ctrl->objtype);
                    }
                }

                /* Update the cwhere vector */
                for(j = cptr[cid]; j < cptr[cid + 1]; j++)
                    cwhere[cind[j]] = target;

                todo[i] = todo[--ntodo];
            }
            if(oldntodo == ntodo)
            {
                IFSET(ctrl->dbglvl,
                      METIS_DBG_CONTIGINFO,
                      printf("Stopped at ntodo: %" PRIDX "\n", ntodo));
                break;
            }
        }

        for(i = 0; i < nvtxs; i++)
            ASSERT(where[i] == cwhere[i]);
    }

    WCOREPOP;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo
 */
/*************************************************************************/
void MoveGroupContigForCut(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t gid, idx_t* ptr, idx_t* ind)
{
    idx_t      i, ii, iii, j, jj, k, l, nvtxs, nbnd, from, me;
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

    for(iii = ptr[gid]; iii < ptr[gid + 1]; iii++)
    {
        i    = ind[iii];
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
            mynbrs[k].pid = to;
            mynbrs[k].ed  = 0;
            myrinfo->nnbrs++;
        }

        graph->mincut -= mynbrs[k].ed - myrinfo->id;

        /* Update ID/ED and BND related information for the moved vertex */
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
        }

        ASSERT(CheckRInfo(ctrl, graph->ckrinfo + i));
    }

    graph->nbnd = nbnd;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo
 */
/*************************************************************************/
void MoveGroupContigForVol(ctrl_t*  ctrl,
                           graph_t* graph,
                           idx_t    to,
                           idx_t    gid,
                           idx_t*   ptr,
                           idx_t*   ind,
                           idx_t*   vmarker,
                           idx_t*   pmarker,
                           idx_t*   modind)
{
    idx_t      i, ii, iii, j, jj, k, l, nvtxs, from, me, other, xgain;
    idx_t *    xadj, *vsize, *adjncy, *where;
    vkrinfo_t *myrinfo, *orinfo;
    vnbr_t *   mynbrs, *onbrs;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vsize  = graph->vsize;
    adjncy = graph->adjncy;
    where  = graph->where;

    for(iii = ptr[gid]; iii < ptr[gid + 1]; iii++)
    {
        i    = ind[iii];
        from = where[i];

        myrinfo = graph->vkrinfo + i;
        if(myrinfo->inbr == -1)
        {
            myrinfo->inbr  = vnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
            myrinfo->nnbrs = 0;
        }
        mynbrs = ctrl->vnbrpool + myrinfo->inbr;

        xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? vsize[i] : 0);

        /* find the location of 'to' in myrinfo or create it if it is not there */
        for(k = 0; k < myrinfo->nnbrs; k++)
        {
            if(mynbrs[k].pid == to)
                break;
        }
        if(k == myrinfo->nnbrs)
        {
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
        }
        else
        {
            graph->minvol -= (xgain + mynbrs[k].gv);
            graph->mincut -= mynbrs[k].ned - myrinfo->nid;
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

        /* Update the id/ed/gains/bnd of potentially affected nodes */
        KWayVolUpdate(ctrl, graph, i, from, to, NULL, NULL, NULL, NULL, NULL, BNDTYPE_REFINE, vmarker, pmarker, modind);

        /*CheckKWayVolPartitionParams(ctrl, graph);*/
    }

    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERTP(ComputeVolume(graph, where) == graph->minvol,
            ("%" PRIDX " %" PRIDX "\n", ComputeVolume(graph, where), graph->minvol));
}
