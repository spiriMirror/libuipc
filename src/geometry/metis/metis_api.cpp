/* Derived from METIS 5.2.1 and GKlib and adapted/reorganized for libuipc.
 * See LICENSE-METIS and LICENSE-GKlib in this directory.
 * This file is split from the port's merged implementation.
 */

#include <metis.h>

/* The merged implementation removes unused source files and GK_MK* macro
 * instantiations while preserving the METIS_PartGraphKway algorithm.
 */

/************************ auxapi.c ************************/
/**
\file
\brief This file contains various helper API routines for using METIS.

\date   Started 5/12/2011
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version\verbatim $Id: auxapi.c 10409 2011-06-25 16:58:34Z karypis $ \endverbatim
*/


/*************************************************************************/
/*! This function frees memory allocated by a METIS public API. */
/*************************************************************************/
int METIS_Free(void* ptr)
{
    if(ptr != NULL)
        free(ptr);
    return METIS_OK;
}


/*************************************************************************/
/*! This function sets the default values for the options.

    \param options points to an array of size at least METIS_NOPTIONS.
*/
/*************************************************************************/
int METIS_SetDefaultOptions(idx_t* options)
{
    iset(METIS_NOPTIONS, -1, options);

    return METIS_OK;
}


/************************ kmetis.c ************************/
/*!
\file
\brief The top-level routines for  multilevel k-way partitioning that minimizes
       the edge cut.

\date   Started 7/28/1997
\author George
\author Copyright 1997-2011, Regents of the University of Minnesota
\version\verbatim $Id: kmetis.c 20398 2016-11-22 17:17:12Z karypis $ \endverbatim
*/


/*************************************************************************/
/*! This function is the entry point for MCKMETIS */
/*************************************************************************/
int METIS_PartGraphKway(idx_t*  nvtxs,
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
                        idx_t*  objval,
                        idx_t*  part)
{
    int      sigrval = 0, renumber = 0;
    graph_t* graph;
    ctrl_t*  ctrl;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;

    /* set up the run parameters */
    ctrl = SetupCtrl(METIS_OP_KMETIS, options, *ncon, *nparts, tpwgts, ubvec);
    if(!ctrl)
    {
        gk_siguntrap();
        return METIS_ERROR_INPUT;
    }

    /* if required, change the numbering to 0 */
    if(ctrl->numflag == 1)
    {
        Change2CNumbering(*nvtxs, xadj, adjncy);
        renumber = 1;
    }

    /* set up the graph */
    graph = SetupGraph(ctrl, *nvtxs, *ncon, xadj, adjncy, vwgt, vsize, adjwgt);

    /* set up multipliers for making balance computations easier */
    SetupKWayBalMultipliers(ctrl, graph);

    /* set various run parameters that depend on the graph */
    ctrl->CoarsenTo =
        gk_max((*nvtxs) / (40 * gk_max(gk_log2(*nparts), 1)), 30 * (*nparts));
    ctrl->nIparts = (ctrl->nIparts != -1 ? ctrl->nIparts :
                                           (ctrl->CoarsenTo == 30 * (*nparts) ? 4 : 5));

    /* take care contiguity requests for disconnected graphs */
    if(ctrl->contig && !IsConnected(graph, 0))
        gk_errexit(SIGERR, "METIS Error: A contiguous partition is requested for a non-contiguous input graph.\n");

    /* allocate workspace memory */
    AllocateWorkSpace(ctrl, graph);

    /* start the partitioning */
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, InitTimers(ctrl));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->TotalTmr));

    iset(*nvtxs, 0, part);
    if(ctrl->dbglvl & 512)
        *objval = (*nparts == 1 ? 0 : BlockKWayPartitioning(ctrl, graph, part));
    else
        *objval = (*nparts == 1 ? 0 : MlevelKWayPartitioning(ctrl, graph, part));

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->TotalTmr));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, PrintTimers(ctrl));

    /* clean up */
    FreeCtrl(&ctrl);

SIGTHROW:
    /* if required, change the numbering back to 1 */
    if(renumber)
        Change2FNumbering(*nvtxs, xadj, adjncy, part);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    return metis_rcode(sigrval);
}


/*************************************************************************/
/*! This function computes a k-way partitioning of a graph that minimizes
    the specified objective function.

    \param ctrl is the control structure
    \param graph is the graph to be partitioned
    \param part is the vector that on return will store the partitioning

    \returns the objective value of the partitioning. The partitioning
             itself is stored in the part vector.
*/
/*************************************************************************/
idx_t MlevelKWayPartitioning(ctrl_t* ctrl, graph_t* graph, idx_t* part)
{
    idx_t    i, j, objval = 0, curobj = 0, bestobj = 0;
    real_t   curbal = 0.0, bestbal = 0.0;
    graph_t* cgraph;
    int      status;


    for(i = 0; i < ctrl->ncuts; i++)
    {
        cgraph = CoarsenGraph(ctrl, graph);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->InitPartTmr));
        AllocateKWayPartitionMemory(ctrl, cgraph);

        /* Release the work space */
        FreeWorkSpace(ctrl);

        /* Compute the initial partitioning */
        InitKWayPartitioning(ctrl, cgraph);

        /* Re-allocate the work space */
        AllocateWorkSpace(ctrl, graph);
        AllocateRefinementWorkSpace(ctrl, graph->nedges, 2 * cgraph->nedges);

        IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->InitPartTmr));
        IFSET(ctrl->dbglvl,
              METIS_DBG_IPART,
              printf("Initial %" PRIDX "-way partitioning cut: %" PRIDX "\n", ctrl->nparts, objval));

        RefineKWay(ctrl, graph, cgraph);

        switch(ctrl->objtype)
        {
            case METIS_OBJTYPE_CUT:
                curobj = graph->mincut;
                break;

            case METIS_OBJTYPE_VOL:
                curobj = graph->minvol;
                break;

            default:
                gk_errexit(SIGERR, "Unknown objtype: %d\n", ctrl->objtype);
        }

        curbal = ComputeLoadImbalanceDiff(graph, ctrl->nparts, ctrl->pijbm, ctrl->ubfactors);

        if(i == 0 || (curbal <= 0.0005 && bestobj > curobj)
           || (bestbal > 0.0005 && curbal < bestbal))
        {
            icopy(graph->nvtxs, graph->where, part);
            bestobj = curobj;
            bestbal = curbal;
        }

        FreeRData(graph);

        if(bestobj == 0)
            break;
    }

    FreeGraph(&graph);

    return bestobj;
}


/*************************************************************************/
/*! This function computes the initial k-way partitioning using PMETIS
*/
/*************************************************************************/
void InitKWayPartitioning(ctrl_t* ctrl, graph_t* graph)
{
    idx_t   i, ntrials, options[METIS_NOPTIONS], curobj = 0, bestobj = 0;
    idx_t*  bestwhere = NULL;
    real_t* ubvec     = NULL;
    int     status;

    METIS_SetDefaultOptions(options);
    //options[METIS_OPTION_NITER]     = 10;
    options[METIS_OPTION_NITER]     = ctrl->niter;
    options[METIS_OPTION_OBJTYPE]   = METIS_OBJTYPE_CUT;
    options[METIS_OPTION_NO2HOP]    = ctrl->no2hop;
    options[METIS_OPTION_ONDISK]    = ctrl->ondisk;
    options[METIS_OPTION_DROPEDGES] = ctrl->dropedges;
    //options[METIS_OPTION_DBGLVL]    = ctrl->dbglvl;

    ubvec = rmalloc(graph->ncon, "InitKWayPartitioning: ubvec");
    for(i = 0; i < graph->ncon; i++)
        ubvec[i] = (real_t)pow(ctrl->ubfactors[i], 1.0 / log(ctrl->nparts));


    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT:
        case METIS_OBJTYPE_VOL:
            options[METIS_OPTION_NCUTS] = ctrl->nIparts;
            status = METIS_PartGraphRecursive(&graph->nvtxs,
                                              &graph->ncon,
                                              graph->xadj,
                                              graph->adjncy,
                                              graph->vwgt,
                                              graph->vsize,
                                              graph->adjwgt,
                                              &ctrl->nparts,
                                              ctrl->tpwgts,
                                              ubvec,
                                              options,
                                              &curobj,
                                              graph->where);

            if(status != METIS_OK)
                gk_errexit(SIGERR, "Failed during initial partitioning\n");

            break;

#ifdef XXX /* This does not seem to help */
        case METIS_OBJTYPE_VOL:
            bestwhere = imalloc(graph->nvtxs, "InitKWayPartitioning: bestwhere");
            options[METIS_OPTION_NCUTS] = 2;

            ntrials = (ctrl->nIparts + 1) / 2;
            for(i = 0; i < ntrials; i++)
            {
                status = METIS_PartGraphRecursive(&graph->nvtxs,
                                                  &graph->ncon,
                                                  graph->xadj,
                                                  graph->adjncy,
                                                  graph->vwgt,
                                                  graph->vsize,
                                                  graph->adjwgt,
                                                  &ctrl->nparts,
                                                  ctrl->tpwgts,
                                                  ubvec,
                                                  options,
                                                  &curobj,
                                                  graph->where);
                if(status != METIS_OK)
                    gk_errexit(SIGERR, "Failed during initial partitioning\n");

                curobj = ComputeVolume(graph, graph->where);

                if(i == 0 || bestobj > curobj)
                {
                    bestobj = curobj;
                    if(i < ntrials - 1)
                        icopy(graph->nvtxs, graph->where, bestwhere);
                }

                if(bestobj == 0)
                    break;
            }
            if(bestobj != curobj)
                icopy(graph->nvtxs, bestwhere, graph->where);

            break;
#endif

        default:
            gk_errexit(SIGERR, "Unknown objtype: %d\n", ctrl->objtype);
    }

    gk_free((void**)&ubvec, &bestwhere, LTERM);
}


/*************************************************************************/
/*! This function computes a k-way partitioning of a graph that minimizes
    the specified objective function.

    \param ctrl is the control structure
    \param graph is the graph to be partitioned
    \param part is the vector that on return will store the partitioning

    \returns the objective value of the partitioning. The partitioning
             itself is stored in the part vector.
*/
/*************************************************************************/
idx_t BlockKWayPartitioning(ctrl_t* ctrl, graph_t* graph, idx_t* part)
{
    idx_t  i, ii, j, nvtxs, objval = 0;
    idx_t* vwgt;
    idx_t  nparts, mynparts;
    idx_t *fpwgts, *cpwgts, *fpart, *perm;
    ipq_t* queue;

    WCOREPUSH;

    nvtxs = graph->nvtxs;
    vwgt  = graph->vwgt;

    nparts = ctrl->nparts;

    mynparts = gk_min(100 * nparts, sqrt(nvtxs));

    for(i = 0; i < nvtxs; i++)
        part[i] = i % nparts;
    irandArrayPermute(nvtxs, part, 4 * nvtxs, 0);
    printf("Random cut: %d\n", (int)ComputeCut(graph, part));

    /* create the initial multi-section */
    mynparts = GrowMultisection(ctrl, graph, mynparts, part);

    /* balance using label-propagation and refine using a randomized greedy strategy */
    BalanceAndRefineLP(ctrl, graph, mynparts, part);

    /* determine the size of the fine partitions */
    fpwgts = iset(mynparts, 0, iwspacemalloc(ctrl, mynparts));
    for(i = 0; i < nvtxs; i++)
        fpwgts[part[i]] += vwgt[i];

    /* create and initialize the queue that will determine
     where to put the next one */
    cpwgts = iset(nparts, 0, iwspacemalloc(ctrl, nparts));
    queue  = ipqCreate(nparts);
    for(i = 0; i < nparts; i++)
        ipqInsert(queue, i, 0);

    /* assign the fine partitions into the coarse partitions */
    fpart = iwspacemalloc(ctrl, mynparts);
    perm  = iwspacemalloc(ctrl, mynparts);
    irandArrayPermute(mynparts, perm, mynparts, 1);
    for(ii = 0; ii < mynparts; ii++)
    {
        i        = perm[ii];
        j        = ipqSeeTopVal(queue);
        fpart[i] = j;
        cpwgts[j] += fpwgts[i];
        ipqUpdate(queue, j, -cpwgts[j]);
    }
    ipqDestroy(queue);

    for(i = 0; i < nparts; i++)
        printf("cpwgts[%d] = %d\n", (int)i, (int)cpwgts[i]);

    for(i = 0; i < nvtxs; i++)
        part[i] = fpart[part[i]];

    WCOREPOP;

    return ComputeCut(graph, part);
}


/*************************************************************************/
/*! This function takes a graph and produces a bisection by using a region
    growing algorithm. The resulting bisection is refined using FM.
    The resulting partition is returned in graph->where.
*/
/*************************************************************************/
idx_t GrowMultisection(ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* where)
{
    idx_t  i, j, k, l, nvtxs, nleft, first, last;
    idx_t *xadj, *vwgt, *adjncy;
    idx_t* queue;
    idx_t  tvwgt, maxpwgt, *pwgts;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->xadj;
    adjncy = graph->adjncy;

    queue = iwspacemalloc(ctrl, nvtxs);


    /* Select the seeds for the nparts-way BFS */
    for(nleft = 0, i = 0; i < nvtxs; i++)
    {
        if(xadj[i + 1] - xadj[i] > 1) /* a seed's degree should be > 1 */
            where[nleft++] = i;
    }
    nparts = gk_min(nparts, nleft);
    for(i = 0; i < nparts; i++)
    {
        j        = irandInRange(nleft);
        queue[i] = where[j];
        where[j] = --nleft;
    }

    pwgts   = iset(nparts, 0, iwspacemalloc(ctrl, nparts));
    tvwgt   = isum(nvtxs, vwgt, 1);
    maxpwgt = (1.5 * tvwgt) / nparts;

    iset(nvtxs, -1, where);
    for(i = 0; i < nparts; i++)
    {
        where[queue[i]] = i;
        pwgts[i]        = vwgt[queue[i]];
    }

    first = 0;
    last  = nparts;
    nleft = nvtxs - nparts;


    /* Start the BFS from queue to get a partition */
    while(first < last)
    {
        i = queue[first++];
        l = where[i];
        if(pwgts[l] > maxpwgt)
            continue;

        for(j = xadj[i]; j < xadj[i + 1]; j++)
        {
            k = adjncy[j];
            if(where[k] == -1)
            {
                if(pwgts[l] + vwgt[k] > maxpwgt)
                    break;
                pwgts[l] += vwgt[k];
                where[k]      = l;
                queue[last++] = k;
                nleft--;
            }
        }
    }

    /* Assign the unassigned vertices randomly to the nparts partitions */
    if(nleft > 0)
    {
        for(i = 0; i < nvtxs; i++)
        {
            if(where[i] == -1)
                where[i] = irandInRange(nparts);
        }
    }

    WCOREPOP;

    return nparts;
}


/*************************************************************************/
/*! This function balances the partitioning using label propagation.
*/
/*************************************************************************/
void BalanceAndRefineLP(ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* where)
{
    idx_t  ii, i, j, k, u, v, nvtxs, iter;
    idx_t *xadj, *vwgt, *adjncy, *adjwgt;
    idx_t  tvwgt, *pwgts, maxpwgt, minpwgt;
    idx_t* perm;
    idx_t  from, to, nmoves, nnbrs, *nbrids, *nbrwgts, *nbrmrks;
    real_t ubfactor;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;

    pwgts = iset(nparts, 0, iwspacemalloc(ctrl, nparts));

    ubfactor = I2RUBFACTOR(ctrl->ufactor);
    tvwgt    = isum(nvtxs, vwgt, 1);
    maxpwgt  = (ubfactor * tvwgt) / nparts;
    minpwgt  = (1.0 * tvwgt) / (ubfactor * nparts);

    for(i = 0; i < nvtxs; i++)
        pwgts[where[i]] += vwgt[i];

    /* for randomly visiting the vertices */
    perm = iincset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));

    /* for keeping track of adjacent partitions */
    nbrids  = iwspacemalloc(ctrl, nparts);
    nbrwgts = iset(nparts, 0, iwspacemalloc(ctrl, nparts));
    nbrmrks = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

    /* perform a fixed number of balancing LP iterations */
    if(ctrl->dbglvl & METIS_DBG_REFINE)
        printf("BLP: nparts: %" PRIDX ", min-max: [%" PRIDX ", %" PRIDX
               "], bal: %7.4" PRREAL ", cut: %9" PRIDX "\n",
               nparts,
               minpwgt,
               maxpwgt,
               1.0 * imax(nparts, pwgts, 1) * nparts / tvwgt,
               ComputeCut(graph, where));
    for(iter = 0; iter < ctrl->niter; iter++)
    {
        if(imax(nparts, pwgts, 1) * nparts < ubfactor * tvwgt)
            break;

        irandArrayPermute(nvtxs, perm, nvtxs / 8, 1);
        nmoves = 0;

        for(ii = 0; ii < nvtxs; ii++)
        {
            u = perm[ii];

            from = where[u];
            if(pwgts[from] - vwgt[u] < minpwgt)
                continue;

            nnbrs = 0;
            for(j = xadj[u]; j < xadj[u + 1]; j++)
            {
                v  = adjncy[j];
                to = where[v];

                if(pwgts[to] + vwgt[u] > maxpwgt)
                    continue; /* skip if 'to' is overweight */

                if((k = nbrmrks[to]) == -1)
                {
                    nbrmrks[to] = k = nnbrs++;
                    nbrids[k]       = to;
                }
                nbrwgts[k] += xadj[v + 1] - xadj[v];
            }
            if(nnbrs == 0)
                continue;

            to = nbrids[iargmax(nnbrs, nbrwgts, 1)];
            if(from != to)
            {
                where[u] = to;
                INC_DEC(pwgts[to], pwgts[from], vwgt[u]);
                nmoves++;
            }

            for(k = 0; k < nnbrs; k++)
            {
                nbrmrks[nbrids[k]] = -1;
                nbrwgts[k]         = 0;
            }
        }

        if(ctrl->dbglvl & METIS_DBG_REFINE)
            printf("     nmoves: %8" PRIDX ", bal: %7.4" PRREAL ", cut: %9" PRIDX "\n",
                   nmoves,
                   1.0 * imax(nparts, pwgts, 1) * nparts / tvwgt,
                   ComputeCut(graph, where));

        if(nmoves == 0)
            break;
    }

    /* perform a fixed number of refinement LP iterations */
    if(ctrl->dbglvl & METIS_DBG_REFINE)
        printf("RLP: nparts: %" PRIDX ", min-max: [%" PRIDX ", %" PRIDX
               "], bal: %7.4" PRREAL ", cut: %9" PRIDX "\n",
               nparts,
               minpwgt,
               maxpwgt,
               1.0 * imax(nparts, pwgts, 1) * nparts / tvwgt,
               ComputeCut(graph, where));
    for(iter = 0; iter < ctrl->niter; iter++)
    {
        irandArrayPermute(nvtxs, perm, nvtxs / 8, 1);
        nmoves = 0;

        for(ii = 0; ii < nvtxs; ii++)
        {
            u = perm[ii];

            from = where[u];
            if(pwgts[from] - vwgt[u] < minpwgt)
                continue;

            nnbrs = 0;
            for(j = xadj[u]; j < xadj[u + 1]; j++)
            {
                v  = adjncy[j];
                to = where[v];

                if(to != from && pwgts[to] + vwgt[u] > maxpwgt)
                    continue; /* skip if 'to' is overweight */

                if((k = nbrmrks[to]) == -1)
                {
                    nbrmrks[to] = k = nnbrs++;
                    nbrids[k]       = to;
                }
                nbrwgts[k] += adjwgt[j];
            }
            if(nnbrs == 0)
                continue;

            to = nbrids[iargmax(nnbrs, nbrwgts, 1)];
            if(from != to)
            {
                where[u] = to;
                INC_DEC(pwgts[to], pwgts[from], vwgt[u]);
                nmoves++;
            }

            for(k = 0; k < nnbrs; k++)
            {
                nbrmrks[nbrids[k]] = -1;
                nbrwgts[k]         = 0;
            }
        }

        if(ctrl->dbglvl & METIS_DBG_REFINE)
            printf("     nmoves: %8" PRIDX ", bal: %7.4" PRREAL ", cut: %9" PRIDX "\n",
                   nmoves,
                   1.0 * imax(nparts, pwgts, 1) * nparts / tvwgt,
                   ComputeCut(graph, where));

        if(nmoves == 0)
            break;
    }

    WCOREPOP;
}
/************************ options.c ************************/
/**
  \file
  \brief This file contains various routines for dealing with options and ctrl_t.

  \date   Started 5/12/2011
  \author George
  \author Copyright 1997-2011, Regents of the University of Minnesota
  \version\verbatim $Id: options.c 17717 2014-10-03 19:09:31Z dominique $ \endverbatim
  */


/*************************************************************************/
/*! This function creates and sets the run parameters (ctrl_t) */
/*************************************************************************/
ctrl_t* SetupCtrl(moptype_et optype, idx_t* options, idx_t ncon, idx_t nparts, real_t* tpwgts, real_t* ubvec)
{
    idx_t   i, j;
    ctrl_t* ctrl;

    ctrl = (ctrl_t*)gk_malloc(sizeof(ctrl_t), "SetupCtrl: ctrl");

    memset((void*)ctrl, 0, sizeof(ctrl_t));

    ctrl->pid = getpid();

    switch(optype)
    {
        case METIS_OP_PMETIS:
            ctrl->objtype = GETOPTION(options, METIS_OPTION_OBJTYPE, METIS_OBJTYPE_CUT);
            ctrl->rtype = METIS_RTYPE_FM;
            ctrl->ncuts = GETOPTION(options, METIS_OPTION_NCUTS, 1);
            ctrl->niter = GETOPTION(options, METIS_OPTION_NITER, 10);

            if(ncon == 1)
            {
                ctrl->iptype = GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_GROW);
                ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, PMETIS_DEFAULT_UFACTOR);
                ctrl->CoarsenTo = 20;
            }
            else
            {
                ctrl->iptype = GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_RANDOM);
                ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, MCPMETIS_DEFAULT_UFACTOR);
                ctrl->CoarsenTo = 100;
            }

            break;


        case METIS_OP_KMETIS:
            ctrl->objtype = GETOPTION(options, METIS_OPTION_OBJTYPE, METIS_OBJTYPE_CUT);
            ctrl->iptype = GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_METISRB);
            ctrl->rtype   = METIS_RTYPE_GREEDY;
            ctrl->nIparts = GETOPTION(options, METIS_OPTION_NIPARTS, -1);
            ctrl->ncuts   = GETOPTION(options, METIS_OPTION_NCUTS, 1);
            ctrl->niter   = GETOPTION(options, METIS_OPTION_NITER, 10);
            ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, KMETIS_DEFAULT_UFACTOR);
            ctrl->minconn = GETOPTION(options, METIS_OPTION_MINCONN, 0);
            ctrl->contig  = GETOPTION(options, METIS_OPTION_CONTIG, 0);
            break;


        case METIS_OP_OMETIS:
            ctrl->objtype = GETOPTION(options, METIS_OPTION_OBJTYPE, METIS_OBJTYPE_NODE);
            ctrl->rtype = GETOPTION(options, METIS_OPTION_RTYPE, METIS_RTYPE_SEP1SIDED);
            ctrl->iptype = GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_EDGE);
            ctrl->nseps = GETOPTION(options, METIS_OPTION_NSEPS, 1);
            ctrl->niter = GETOPTION(options, METIS_OPTION_NITER, 10);
            ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, OMETIS_DEFAULT_UFACTOR);
            ctrl->compress = GETOPTION(options, METIS_OPTION_COMPRESS, 1);
            ctrl->ccorder  = GETOPTION(options, METIS_OPTION_CCORDER, 0);
            ctrl->pfactor  = 0.1 * GETOPTION(options, METIS_OPTION_PFACTOR, 0);

            ctrl->CoarsenTo = 100;
            break;

        default:
            gk_errexit(SIGERR, "Unknown optype of %d\n", optype);
    }

    /* common options */
    ctrl->ctype     = GETOPTION(options, METIS_OPTION_CTYPE, METIS_CTYPE_SHEM);
    ctrl->no2hop    = GETOPTION(options, METIS_OPTION_NO2HOP, 0);
    ctrl->ondisk    = GETOPTION(options, METIS_OPTION_ONDISK, 0);
    ctrl->seed      = GETOPTION(options, METIS_OPTION_SEED, -1);
    ctrl->dbglvl    = GETOPTION(options, METIS_OPTION_DBGLVL, 0);
    ctrl->numflag   = GETOPTION(options, METIS_OPTION_NUMBERING, 0);
    ctrl->dropedges = GETOPTION(options, METIS_OPTION_DROPEDGES, 0);

    /* set non-option information */
    ctrl->optype  = optype;
    ctrl->ncon    = ncon;
    ctrl->nparts  = nparts;
    ctrl->maxvwgt = ismalloc(ncon, 0, "SetupCtrl: maxvwgt");

    /* setup the target partition weights */
    if(ctrl->optype != METIS_OP_OMETIS)
    {
        ctrl->tpwgts = rsmalloc(nparts * ncon, 0.0, "SetupCtrl: ctrl->tpwgts");
        if(tpwgts)
        {
            rcopy(nparts * ncon, tpwgts, ctrl->tpwgts);
        }
        else
        {
            for(i = 0; i < nparts; i++)
            {
                for(j = 0; j < ncon; j++)
                    ctrl->tpwgts[i * ncon + j] = 1.0 / nparts;
            }
        }
    }
    else
    { /* METIS_OP_OMETIS */
        /* this is required to allow the pijbm to be defined properly for
       the edge-based refinement during initial partitioning */
        ctrl->tpwgts = rsmalloc(2, .5, "SetupCtrl: ctrl->tpwgts");
    }


    /* setup the ubfactors */
    ctrl->ubfactors =
        rsmalloc(ctrl->ncon, I2RUBFACTOR(ctrl->ufactor), "SetupCtrl: ubfactors");
    if(ubvec)
        rcopy(ctrl->ncon, ubvec, ctrl->ubfactors);
    for(i = 0; i < ctrl->ncon; i++)
        ctrl->ubfactors[i] += 0.0000499;

    /* Allocate memory for balance multipliers.
     Note that for PMETIS/OMETIS routines the memory allocated is more
     than required as balance multipliers for 2 parts is sufficient. */
    ctrl->pijbm = rmalloc(nparts * ncon, "SetupCtrl: ctrl->pijbm");

    InitRandom(ctrl->seed);

    IFSET(ctrl->dbglvl, METIS_DBG_INFO, PrintCtrl(ctrl));

    if(!CheckParams(ctrl))
    {
        FreeCtrl(&ctrl);
        return NULL;
    }
    else
    {
        return ctrl;
    }
}


/*************************************************************************/
/*! Computes the per-partition/constraint balance multipliers */
/*************************************************************************/
void SetupKWayBalMultipliers(ctrl_t* ctrl, graph_t* graph)
{
    idx_t i, j;

    for(i = 0; i < ctrl->nparts; i++)
    {
        for(j = 0; j < graph->ncon; j++)
            ctrl->pijbm[i * graph->ncon + j] =
                graph->invtvwgt[j] / ctrl->tpwgts[i * graph->ncon + j];
    }
}


/*************************************************************************/
/*! Computes the per-partition/constraint balance multipliers */
/*************************************************************************/
void Setup2WayBalMultipliers(ctrl_t* ctrl, graph_t* graph, real_t* tpwgts)
{
    idx_t i, j;

    for(i = 0; i < 2; i++)
    {
        for(j = 0; j < graph->ncon; j++)
            ctrl->pijbm[i * graph->ncon + j] =
                graph->invtvwgt[j] / tpwgts[i * graph->ncon + j];
    }
}


/*************************************************************************/
/*! This function prints the various control fields */
/*************************************************************************/
void PrintCtrl(ctrl_t* ctrl)
{
    idx_t i, j, modnum;

    printf(" Runtime parameters:\n");

    printf("   Objective type: ");
    switch(ctrl->objtype)
    {
        case METIS_OBJTYPE_CUT:
            printf("METIS_OBJTYPE_CUT\n");
            break;
        case METIS_OBJTYPE_VOL:
            printf("METIS_OBJTYPE_VOL\n");
            break;
        case METIS_OBJTYPE_NODE:
            printf("METIS_OBJTYPE_NODE\n");
            break;
        default:
            printf("Unknown!\n");
    }

    printf("   Coarsening type: ");
    switch(ctrl->ctype)
    {
        case METIS_CTYPE_RM:
            printf("METIS_CTYPE_RM\n");
            break;
        case METIS_CTYPE_SHEM:
            printf("METIS_CTYPE_SHEM\n");
            break;
        default:
            printf("Unknown!\n");
    }

    printf("   Initial partitioning type: ");
    switch(ctrl->iptype)
    {
        case METIS_IPTYPE_GROW:
            printf("METIS_IPTYPE_GROW\n");
            break;
        case METIS_IPTYPE_RANDOM:
            printf("METIS_IPTYPE_RANDOM\n");
            break;
        case METIS_IPTYPE_EDGE:
            printf("METIS_IPTYPE_EDGE\n");
            break;
        case METIS_IPTYPE_NODE:
            printf("METIS_IPTYPE_NODE\n");
            break;
        case METIS_IPTYPE_METISRB:
            printf("METIS_IPTYPE_METISRB\n");
            break;
        default:
            printf("Unknown!\n");
    }

    printf("   Refinement type: ");
    switch(ctrl->rtype)
    {
        case METIS_RTYPE_FM:
            printf("METIS_RTYPE_FM\n");
            break;
        case METIS_RTYPE_GREEDY:
            printf("METIS_RTYPE_GREEDY\n");
            break;
        case METIS_RTYPE_SEP2SIDED:
            printf("METIS_RTYPE_SEP2SIDED\n");
            break;
        case METIS_RTYPE_SEP1SIDED:
            printf("METIS_RTYPE_SEP1SIDED\n");
            break;
        default:
            printf("Unknown!\n");
    }

    printf("   Perform a 2-hop matching: %s\n", (ctrl->no2hop ? "No" : "Yes"));

    printf("   On disk storage: %s\n", (ctrl->ondisk ? "Yes" : "No"));
    printf("   Drop edges: %s\n", (ctrl->dropedges ? "Yes" : "No"));

    printf("   Number of balancing constraints: %" PRIDX "\n", ctrl->ncon);
    printf("   Number of refinement iterations: %" PRIDX "\n", ctrl->niter);
    printf("   Number of initial partitionings: %" PRIDX "\n", ctrl->nIparts);
    printf("   Random number seed: %" PRIDX "\n", ctrl->seed);

    if(ctrl->optype == METIS_OP_OMETIS)
    {
        printf("   Number of separators: %" PRIDX "\n", ctrl->nseps);
        printf("   Compress graph prior to ordering: %s\n", (ctrl->compress ? "Yes" : "No"));
        printf("   Detect & order connected components separately: %s\n",
               (ctrl->ccorder ? "Yes" : "No"));
        printf("   Prunning factor for high degree vertices: %" PRREAL "\n", ctrl->pfactor);
    }
    else
    {
        printf("   Number of partitions: %" PRIDX "\n", ctrl->nparts);
        printf("   Number of cuts: %" PRIDX "\n", ctrl->ncuts);
        printf("   User-supplied ufactor: %" PRIDX "\n", ctrl->ufactor);

        if(ctrl->optype == METIS_OP_KMETIS)
        {
            printf("   Minimize connectivity: %s\n", (ctrl->minconn ? "Yes" : "No"));
            printf("   Create contiguous partitions: %s\n", (ctrl->contig ? "Yes" : "No"));
        }

        modnum = (ctrl->ncon == 1 ? 5 : (ctrl->ncon == 2 ? 3 : (ctrl->ncon == 3 ? 2 : 1)));
        printf("   Target partition weights: ");
        for(i = 0; i < ctrl->nparts; i++)
        {
            if(i % modnum == 0)
                printf("\n     ");
            printf("%4" PRIDX "=[", i);
            for(j = 0; j < ctrl->ncon; j++)
                printf("%s%.2e", (j == 0 ? "" : " "), (double)ctrl->tpwgts[i * ctrl->ncon + j]);
            printf("]");
        }
        printf("\n");
    }

    printf("   Allowed maximum load imbalance: ");
    for(i = 0; i < ctrl->ncon; i++)
        printf("%.3" PRREAL " ", ctrl->ubfactors[i]);
    printf("\n");

    printf("\n");
}


/*************************************************************************/
/*! This function checks the validity of user-supplied parameters */
/*************************************************************************/
int CheckParams(ctrl_t* ctrl)
{
    idx_t      i, j;
    real_t     sum;
    mdbglvl_et dbglvl = METIS_DBG_INFO;

    switch(ctrl->optype)
    {
        case METIS_OP_PMETIS:
            if(ctrl->objtype != METIS_OBJTYPE_CUT)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect objective type.\n"));
                return 0;
            }
            if(ctrl->ctype != METIS_CTYPE_RM && ctrl->ctype != METIS_CTYPE_SHEM)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect coarsening scheme.\n"));
                return 0;
            }
            if(ctrl->iptype != METIS_IPTYPE_GROW && ctrl->iptype != METIS_IPTYPE_RANDOM)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect initial partitioning scheme.\n"));
                return 0;
            }
            if(ctrl->rtype != METIS_RTYPE_FM)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect refinement scheme.\n"));
                return 0;
            }
            if(ctrl->ncuts <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncuts.\n"));
                return 0;
            }
            if(ctrl->niter <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect niter.\n"));
                return 0;
            }
            if(ctrl->ufactor <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ufactor.\n"));
                return 0;
            }
            if(ctrl->numflag != 0 && ctrl->numflag != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect numflag.\n"));
                return 0;
            }
            if(ctrl->nparts <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nparts.\n"));
                return 0;
            }
            if(ctrl->ncon <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncon.\n"));
                return 0;
            }

            for(i = 0; i < ctrl->ncon; i++)
            {
                sum = rsum(ctrl->nparts, ctrl->tpwgts + i, ctrl->ncon);
                if(sum < 0.99 || sum > 1.01)
                {
                    IFSET(dbglvl,
                          METIS_DBG_INFO,
                          printf("Input Error: Incorrect sum of %" PRREAL
                                 " for tpwgts for constraint %" PRIDX ".\n",
                                 sum,
                                 i));
                    return 0;
                }
            }
            for(i = 0; i < ctrl->ncon; i++)
            {
                for(j = 0; j < ctrl->nparts; j++)
                {
                    if(ctrl->tpwgts[j * ctrl->ncon + i] <= 0.0)
                    {
                        IFSET(dbglvl,
                              METIS_DBG_INFO,
                              printf("Input Error: Incorrect tpwgts for partition %" PRIDX
                                     " and constraint %" PRIDX ".\n",
                                     j,
                                     i));
                        return 0;
                    }
                }
            }

            for(i = 0; i < ctrl->ncon; i++)
            {
                if(ctrl->ubfactors[i] <= 1.0)
                {
                    IFSET(dbglvl,
                          METIS_DBG_INFO,
                          printf("Input Error: Incorrect ubfactor for constraint %" PRIDX ".\n",
                                 i));
                    return 0;
                }
            }

            break;

        case METIS_OP_KMETIS:
            if(ctrl->objtype != METIS_OBJTYPE_CUT && ctrl->objtype != METIS_OBJTYPE_VOL)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect objective type.\n"));
                return 0;
            }
            if(ctrl->ctype != METIS_CTYPE_RM && ctrl->ctype != METIS_CTYPE_SHEM)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect coarsening scheme.\n"));
                return 0;
            }
            if(ctrl->iptype != METIS_IPTYPE_METISRB && ctrl->iptype != METIS_IPTYPE_GROW)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect initial partitioning scheme.\n"));
                return 0;
            }
            if(ctrl->rtype != METIS_RTYPE_GREEDY)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect refinement scheme.\n"));
                return 0;
            }
            if(ctrl->ncuts <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncuts.\n"));
                return 0;
            }
            if(ctrl->niter <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect niter.\n"));
                return 0;
            }
            if(ctrl->ufactor <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ufactor.\n"));
                return 0;
            }
            if(ctrl->numflag != 0 && ctrl->numflag != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect numflag.\n"));
                return 0;
            }
            if(ctrl->nparts <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nparts.\n"));
                return 0;
            }
            if(ctrl->ncon <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncon.\n"));
                return 0;
            }
            if(ctrl->contig != 0 && ctrl->contig != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect contig.\n"));
                return 0;
            }
            if(ctrl->minconn != 0 && ctrl->minconn != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect minconn.\n"));
                return 0;
            }

            for(i = 0; i < ctrl->ncon; i++)
            {
                sum = rsum(ctrl->nparts, ctrl->tpwgts + i, ctrl->ncon);
                if(sum < 0.99 || sum > 1.01)
                {
                    IFSET(dbglvl,
                          METIS_DBG_INFO,
                          printf("Input Error: Incorrect sum of %" PRREAL
                                 " for tpwgts for constraint %" PRIDX ".\n",
                                 sum,
                                 i));
                    return 0;
                }
            }
            for(i = 0; i < ctrl->ncon; i++)
            {
                for(j = 0; j < ctrl->nparts; j++)
                {
                    if(ctrl->tpwgts[j * ctrl->ncon + i] <= 0.0)
                    {
                        IFSET(dbglvl,
                              METIS_DBG_INFO,
                              printf("Input Error: Incorrect tpwgts for partition %" PRIDX
                                     " and constraint %" PRIDX ".\n",
                                     j,
                                     i));
                        return 0;
                    }
                }
            }

            for(i = 0; i < ctrl->ncon; i++)
            {
                if(ctrl->ubfactors[i] <= 1.0)
                {
                    IFSET(dbglvl,
                          METIS_DBG_INFO,
                          printf("Input Error: Incorrect ubfactor for constraint %" PRIDX ".\n",
                                 i));
                    return 0;
                }
            }

            break;


        case METIS_OP_OMETIS:
            if(ctrl->objtype != METIS_OBJTYPE_NODE)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect objective type.\n"));
                return 0;
            }
            if(ctrl->ctype != METIS_CTYPE_RM && ctrl->ctype != METIS_CTYPE_SHEM)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect coarsening scheme.\n"));
                return 0;
            }
            if(ctrl->iptype != METIS_IPTYPE_EDGE && ctrl->iptype != METIS_IPTYPE_NODE)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect initial partitioning scheme.\n"));
                return 0;
            }
            if(ctrl->rtype != METIS_RTYPE_SEP1SIDED && ctrl->rtype != METIS_RTYPE_SEP2SIDED)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect refinement scheme.\n"));
                return 0;
            }
            if(ctrl->nseps <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nseps.\n"));
                return 0;
            }
            if(ctrl->niter <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect niter.\n"));
                return 0;
            }
            if(ctrl->ufactor <= 0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ufactor.\n"));
                return 0;
            }
            if(ctrl->numflag != 0 && ctrl->numflag != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect numflag.\n"));
                return 0;
            }
            if(ctrl->nparts != 3)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nparts.\n"));
                return 0;
            }
            if(ctrl->ncon != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncon.\n"));
                return 0;
            }
            if(ctrl->compress != 0 && ctrl->compress != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect compress.\n"));
                return 0;
            }
            if(ctrl->ccorder != 0 && ctrl->ccorder != 1)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ccorder.\n"));
                return 0;
            }
            if(ctrl->pfactor < 0.0)
            {
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect pfactor.\n"));
                return 0;
            }

            for(i = 0; i < ctrl->ncon; i++)
            {
                if(ctrl->ubfactors[i] <= 1.0)
                {
                    IFSET(dbglvl,
                          METIS_DBG_INFO,
                          printf("Input Error: Incorrect ubfactor for constraint %" PRIDX ".\n",
                                 i));
                    return 0;
                }
            }

            break;

        default:
            IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect optype\n"));
            return 0;
    }

    return 1;
}


/*************************************************************************/
/*! This function frees the memory associated with a ctrl_t */
/*************************************************************************/
void FreeCtrl(ctrl_t** r_ctrl)
{
    ctrl_t* ctrl = *r_ctrl;

    FreeWorkSpace(ctrl);

    gk_free((void**)&ctrl->tpwgts, &ctrl->pijbm, &ctrl->ubfactors, &ctrl->maxvwgt, &ctrl, LTERM);

    *r_ctrl = NULL;
}


/************************ pmetis.c ************************/
/**
\file
\brief This file contains the top level routines for the multilevel recursive bisection
       algorithm PMETIS.

\date   Started 7/24/1997
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota
\version\verbatim $Id: pmetis.c 10513 2011-07-07 22:06:03Z karypis $ \endverbatim
*/


/*************************************************************************/
/*! \ingroup api
    \brief Recursive partitioning routine.

    This function computes a partitioning of a graph based on multilevel
    recursive bisection. It can be used to partition a graph into \e k
    parts. The objective of the partitioning is to minimize the edgecut
    subject to one or more balancing constraints.

    \param[in] nvtxs is the number of vertices in the graph.

    \param[in] ncon is the number of balancing constraints. For the standard
           partitioning problem in which each vertex is either unweighted
           or has a single weight, ncon should be 1.

    \param[in] xadj is an array of size nvtxs+1 used to specify the starting
           positions of the adjacency structure of the vertices in the
           adjncy array.

    \param[in] adjncy is an array of size to the sum of the degrees of the
           graph that stores for each vertex the set of vertices that
           is adjacent to.

    \param[in] vwgt is an array of size nvtxs*ncon that stores the weights
           of the vertices for each constraint. The ncon weights for the
           ith vertex are stored in the ncon consecutive locations starting
           at vwgt[i*ncon]. When ncon==1, a NULL value can be passed indicating
           that all the vertices in the graph have the same weight.

    \param[in] adjwgt is an array of size equal to adjncy, specifying the weight
           for each edge (i.e., adjwgt[j] corresponds to the weight of the
           edge stored in adjncy[j]).
           A NULL value can be passed indicating that all the edges in the
           graph have the same weight.

    \param[in] nparts is the number of desired partitions.

    \param[in] tpwgts is an array of size nparts*ncon that specifies the
           desired weight for each part and constraint. The \e{target partition
           weight} for the ith part and jth constraint is specified
           at tpwgts[i*ncon+j] (the numbering of i and j starts from 0).
           For each constraint, the sum of the tpwgts[] entries must be
           1.0 (i.e., \f$ \sum_i tpwgts[i*ncon+j] = 1.0 \f$).
           A NULL value can be passed indicating that the graph should
           be equally divided among the parts.

    \param[in] ubvec is an array of size ncon that specifies the allowed
           load imbalance tolerance for each constraint.
           For the ith part and jth constraint the allowed weight is the
           ubvec[j]*tpwgts[i*ncon+j] fraction of the jth's constraint total
           weight. The load imbalances must be greater than 1.0.
           A NULL value can be passed indicating that the load imbalance
           tolerance for each constraint should be 1.001 (for ncon==1)
           or 1.01 (for ncon>1).

    \params[in] options is the array for passing additional parameters
           in order to customize the behaviour of the partitioning
           algorithm.

    \params[out] edgecut stores the cut of the partitioning.

    \params[out] part is an array of size nvtxs used to store the
           computed partitioning. The partition number for the ith
           vertex is stored in part[i]. Based on the numflag parameter,
           the numbering of the parts starts from either 0 or 1.


    \returns
      \retval METIS_OK  indicates that the function returned normally.
      \retval METIS_ERROR_INPUT indicates an input error.
      \retval METIS_ERROR_MEMORY indicates that it could not allocate
              the required memory.

*/
/*************************************************************************/
int METIS_PartGraphRecursive(idx_t*  nvtxs,
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
                             idx_t*  objval,
                             idx_t*  part)
{
    int      sigrval = 0, renumber = 0;
    graph_t* graph;
    ctrl_t*  ctrl;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;

    /* set up the run parameters */
    ctrl = SetupCtrl(METIS_OP_PMETIS, options, *ncon, *nparts, tpwgts, ubvec);
    if(!ctrl)
    {
        gk_siguntrap();
        return METIS_ERROR_INPUT;
    }

    /* if required, change the numbering to 0 */
    if(ctrl->numflag == 1)
    {
        Change2CNumbering(*nvtxs, xadj, adjncy);
        renumber = 1;
    }

    /* set up the graph */
    graph = SetupGraph(ctrl, *nvtxs, *ncon, xadj, adjncy, vwgt, vsize, adjwgt);

    /* allocate workspace memory */
    AllocateWorkSpace(ctrl, graph);

    /* start the partitioning */
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, InitTimers(ctrl));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->TotalTmr));

    iset(*nvtxs, 0, part);
    *objval = (*nparts == 1 ?
                   0 :
                   MlevelRecursiveBisection(ctrl, graph, *nparts, part, ctrl->tpwgts, 0));

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->TotalTmr));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, PrintTimers(ctrl));

    /* clean up */
    FreeCtrl(&ctrl);

SIGTHROW:
    /* if required, change the numbering back to 1 */
    if(renumber)
        Change2FNumbering(*nvtxs, xadj, adjncy, part);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    return metis_rcode(sigrval);
}


/*************************************************************************/
/*! This function is the top-level driver of the recursive bisection
    routine. */
/*************************************************************************/
idx_t MlevelRecursiveBisection(
    ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* part, real_t* tpwgts, idx_t fpart)
{
    idx_t    i, j, nvtxs, ncon, objval;
    idx_t *  label, *where;
    graph_t *lgraph, *rgraph;
    real_t   wsum, *tpwgts2;

    if((nvtxs = graph->nvtxs) == 0)
    {
        printf(
            "\t***Cannot bisect a graph with 0 vertices!\n"
            "\t***You are trying to partition a graph into too many parts!\n");
        return 0;
    }

    ncon = graph->ncon;

    /* determine the weights of the two partitions as a function of the weight of the
     target partition weights */
    WCOREPUSH;
    tpwgts2 = rwspacemalloc(ctrl, 2 * ncon);
    for(i = 0; i < ncon; i++)
    {
        tpwgts2[i]        = rsum((nparts >> 1), tpwgts + i, ncon);
        tpwgts2[ncon + i] = 1.0 - tpwgts2[i];
    }

    /* perform the bisection */
    objval = MultilevelBisect(ctrl, graph, tpwgts2);

    WCOREPOP;

    label = graph->label;
    where = graph->where;
    for(i = 0; i < nvtxs; i++)
        part[label[i]] = where[i] + fpart;

    if(nparts > 2)
        SplitGraphPart(ctrl, graph, &lgraph, &rgraph);

    /* Free the memory of the top level graph */
    FreeGraph(&graph);

    /* Scale the fractions in the tpwgts according to the true weight */
    for(i = 0; i < ncon; i++)
    {
        wsum = rsum((nparts >> 1), tpwgts + i, ncon);
        rscale((nparts >> 1), 1.0 / wsum, tpwgts + i, ncon);
        rscale(nparts - (nparts >> 1), 1.0 / (1.0 - wsum), tpwgts + (nparts >> 1) * ncon + i, ncon);
    }

    /* Do the recursive call */
    if(nparts > 3)
    {
        objval += MlevelRecursiveBisection(ctrl, lgraph, (nparts >> 1), part, tpwgts, fpart);
        objval += MlevelRecursiveBisection(ctrl,
                                           rgraph,
                                           nparts - (nparts >> 1),
                                           part,
                                           tpwgts + (nparts >> 1) * ncon,
                                           fpart + (nparts >> 1));
    }
    else if(nparts == 3)
    {
        FreeGraph(&lgraph);
        objval += MlevelRecursiveBisection(ctrl,
                                           rgraph,
                                           nparts - (nparts >> 1),
                                           part,
                                           tpwgts + (nparts >> 1) * ncon,
                                           fpart + (nparts >> 1));
    }


    return objval;
}


/*************************************************************************/
/*! This function performs a multilevel bisection */
/*************************************************************************/
idx_t MultilevelBisect(ctrl_t* ctrl, graph_t* graph, real_t* tpwgts)
{
    idx_t    i, niparts, bestobj = 0, curobj = 0, *bestwhere = NULL;
    graph_t* cgraph;
    real_t   bestbal = 0.0, curbal = 0.0;

    Setup2WayBalMultipliers(ctrl, graph, tpwgts);

    WCOREPUSH;

    if(ctrl->ncuts > 1)
        bestwhere = iwspacemalloc(ctrl, graph->nvtxs);

    for(i = 0; i < ctrl->ncuts; i++)
    {
        cgraph = CoarsenGraph(ctrl, graph);

        niparts = (cgraph->nvtxs <= ctrl->CoarsenTo ? SMALLNIPARTS : LARGENIPARTS);
        Init2WayPartition(ctrl, cgraph, tpwgts, niparts);

        Refine2Way(ctrl, graph, cgraph, tpwgts);

        curobj = graph->mincut;
        curbal = ComputeLoadImbalanceDiff(graph, 2, ctrl->pijbm, ctrl->ubfactors);

        if(i == 0 || (curbal <= 0.0005 && bestobj > curobj)
           || (bestbal > 0.0005 && curbal < bestbal))
        {
            bestobj = curobj;
            bestbal = curbal;
            if(i < ctrl->ncuts - 1)
                icopy(graph->nvtxs, graph->where, bestwhere);
        }

        if(bestobj == 0)
            break;

        if(i < ctrl->ncuts - 1)
            FreeRData(graph);
    }

    if(bestobj != curobj)
    {
        icopy(graph->nvtxs, bestwhere, graph->where);
        Compute2WayPartitionParams(ctrl, graph);
    }

    WCOREPOP;

    return bestobj;
}


/*************************************************************************/
/*! This function splits a graph into two based on its bisection */
/*************************************************************************/
void SplitGraphPart(ctrl_t* ctrl, graph_t* graph, graph_t** r_lgraph, graph_t** r_rgraph)
{
    idx_t  i, j, k, l, istart, iend, mypart, nvtxs, ncon, snvtxs[2], snedges[2];
    idx_t *xadj, *vwgt, *adjncy, *adjwgt, *label, *where, *bndptr;
    idx_t *sxadj[2], *svwgt[2], *sadjncy[2], *sadjwgt[2], *slabel[2];
    idx_t* rename;
    idx_t *auxadjncy, *auxadjwgt;
    graph_t *lgraph, *rgraph;

    WCOREPUSH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->SplitTmr));

    nvtxs  = graph->nvtxs;
    ncon   = graph->ncon;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    label  = graph->label;
    where  = graph->where;
    bndptr = graph->bndptr;

    ASSERT(bndptr != NULL);

    rename = iwspacemalloc(ctrl, nvtxs);

    snvtxs[0] = snvtxs[1] = snedges[0] = snedges[1] = 0;
    for(i = 0; i < nvtxs; i++)
    {
        k         = where[i];
        rename[i] = snvtxs[k]++;
        snedges[k] += xadj[i + 1] - xadj[i];
    }

    lgraph     = SetupSplitGraph(graph, snvtxs[0], snedges[0]);
    sxadj[0]   = lgraph->xadj;
    svwgt[0]   = lgraph->vwgt;
    sadjncy[0] = lgraph->adjncy;
    sadjwgt[0] = lgraph->adjwgt;
    slabel[0]  = lgraph->label;

    rgraph     = SetupSplitGraph(graph, snvtxs[1], snedges[1]);
    sxadj[1]   = rgraph->xadj;
    svwgt[1]   = rgraph->vwgt;
    sadjncy[1] = rgraph->adjncy;
    sadjwgt[1] = rgraph->adjwgt;
    slabel[1]  = rgraph->label;

    snvtxs[0] = snvtxs[1] = snedges[0] = snedges[1] = 0;
    sxadj[0][0] = sxadj[1][0] = 0;
    for(i = 0; i < nvtxs; i++)
    {
        mypart = where[i];

        istart = xadj[i];
        iend   = xadj[i + 1];
        if(bndptr[i] == -1)
        { /* This is an interior vertex */
            auxadjncy = sadjncy[mypart] + snedges[mypart] - istart;
            auxadjwgt = sadjwgt[mypart] + snedges[mypart] - istart;
            for(j = istart; j < iend; j++)
            {
                auxadjncy[j] = adjncy[j];
                auxadjwgt[j] = adjwgt[j];
            }
            snedges[mypart] += iend - istart;
        }
        else
        {
            auxadjncy = sadjncy[mypart];
            auxadjwgt = sadjwgt[mypart];
            l         = snedges[mypart];
            for(j = istart; j < iend; j++)
            {
                k = adjncy[j];
                if(where[k] == mypart)
                {
                    auxadjncy[l]   = k;
                    auxadjwgt[l++] = adjwgt[j];
                }
            }
            snedges[mypart] = l;
        }

        /* copy vertex weights */
        for(k = 0; k < ncon; k++)
            svwgt[mypart][snvtxs[mypart] * ncon + k] = vwgt[i * ncon + k];

        slabel[mypart][snvtxs[mypart]]  = label[i];
        sxadj[mypart][++snvtxs[mypart]] = snedges[mypart];
    }

    for(mypart = 0; mypart < 2; mypart++)
    {
        iend      = sxadj[mypart][snvtxs[mypart]];
        auxadjncy = sadjncy[mypart];
        for(i = 0; i < iend; i++)
            auxadjncy[i] = rename[auxadjncy[i]];
    }

    lgraph->nedges = snedges[0];
    rgraph->nedges = snedges[1];

    SetupGraph_tvwgt(lgraph);
    SetupGraph_tvwgt(rgraph);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->SplitTmr));

    *r_lgraph = lgraph;
    *r_rgraph = rgraph;

    WCOREPOP;
}


/************************ fortran.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * fortran.c
 *
 * This file contains code for the fortran to C interface
 *
 * Started 8/19/97
 * George
 *
 */


/*************************************************************************/
/*! This function changes the numbering to start from 0 instead of 1 */
/*************************************************************************/
void Change2CNumbering(idx_t nvtxs, idx_t* xadj, idx_t* adjncy)
{
    idx_t i;

    for(i = 0; i <= nvtxs; i++)
        xadj[i]--;

    for(i = 0; i < xadj[nvtxs]; i++)
        adjncy[i]--;
}


/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void Change2FNumbering(idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vector)
{
    idx_t i;

    for(i = 0; i < nvtxs; i++)
        vector[i]++;

    for(i = 0; i < xadj[nvtxs]; i++)
        adjncy[i]++;

    for(i = 0; i <= nvtxs; i++)
        xadj[i]++;
}


/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void Change2FNumbering2(idx_t nvtxs, idx_t* xadj, idx_t* adjncy)
{
    idx_t i, nedges;

    nedges = xadj[nvtxs];
    for(i = 0; i < nedges; i++)
        adjncy[i]++;

    for(i = 0; i <= nvtxs; i++)
        xadj[i]++;
}


/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void Change2FNumberingOrder(idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* v1, idx_t* v2)
{
    idx_t i, nedges;

    for(i = 0; i < nvtxs; i++)
    {
        v1[i]++;
        v2[i]++;
    }

    nedges = xadj[nvtxs];
    for(i = 0; i < nedges; i++)
        adjncy[i]++;

    for(i = 0; i <= nvtxs; i++)
        xadj[i]++;
}


/*************************************************************************/
/*! This function changes mesh numbering to start from 0 instead of 1 */
/*************************************************************************/
void ChangeMesh2CNumbering(idx_t n, idx_t* ptr, idx_t* ind)
{
    idx_t i;

    for(i = 0; i <= n; i++)
        ptr[i]--;
    for(i = 0; i < ptr[n]; i++)
        ind[i]--;
}


/*************************************************************************/
/*! This function changes mesh and graph numbering to start from 1 */
/*************************************************************************/
void ChangeMesh2FNumbering(idx_t n, idx_t* ptr, idx_t* ind, idx_t nvtxs, idx_t* xadj, idx_t* adjncy)
{
    idx_t i;

    for(i = 0; i < ptr[n]; i++)
        ind[i]++;
    for(i = 0; i <= n; i++)
        ptr[i]++;

    for(i = 0; i < xadj[nvtxs]; i++)
        adjncy[i]++;
    for(i = 0; i <= nvtxs; i++)
        xadj[i]++;
}


/*************************************************************************/
/*! This function changes mesh partitions and numbering to start from 1 */
/*************************************************************************/
void ChangeMesh2FNumbering2(idx_t ne, idx_t nn, idx_t* ptr, idx_t* ind, idx_t* epart, idx_t* npart)
{
    idx_t i;

    for(i = 0; i < ptr[ne]; i++)
        ind[i]++;
    for(i = 0; i <= ne; i++)
        ptr[i]++;

    for(i = 0; i < ne; i++)
        epart[i]++;

    for(i = 0; i < nn; i++)
        npart[i]++;
}
