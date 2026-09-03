/* Derived from METIS 5.2.1 and adapted for libuipc's C++ build.
 * See LICENSE-METIS in this directory.
 * The nested-dissection ordering algorithms remain source-equivalent.
 */

#include <metis.h>

/************************ compress.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * compress.c
 *
 * This file contains code for compressing nodes with identical adjacency
 * structure and for prunning dense columns
 *
 * Started 9/17/97
 * George
 */


/*************************************************************************/
/*! This function compresses a graph by merging identical vertices
    The compression should lead to at least 10% reduction.

    The compressed graph that is generated has its adjwgts set to 1.

    \returns 1 if compression was performed, otherwise it returns 0.

*/
/**************************************************************************/
graph_t* CompressGraph(
    ctrl_t* ctrl, idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* cptr, idx_t* cind)
{
    idx_t    i, ii, iii, j, jj, k, l, cnvtxs, cnedges;
    idx_t *  cxadj, *cadjncy, *cvwgt, *mark, *map;
    ikv_t*   keys;
    graph_t* graph = NULL;

    mark = ismalloc(nvtxs, -1, "CompressGraph: mark");
    map  = ismalloc(nvtxs, -1, "CompressGraph: map");
    keys = ikvmalloc(nvtxs, "CompressGraph: keys");

    /* Compute a key for each adjacency list */
    for(i = 0; i < nvtxs; i++)
    {
        k = 0;
        for(j = xadj[i]; j < xadj[i + 1]; j++)
            k += adjncy[j];
        keys[i].key = k + i; /* Add the diagonal entry as well */
        keys[i].val = i;
    }

    ikvsorti(nvtxs, keys);

    l = cptr[0] = 0;
    for(cnvtxs = i = 0; i < nvtxs; i++)
    {
        ii = keys[i].val;
        if(map[ii] == -1)
        {
            mark[ii] = i; /* Add the diagonal entry */
            for(j = xadj[ii]; j < xadj[ii + 1]; j++)
                mark[adjncy[j]] = i;

            map[ii]   = cnvtxs;
            cind[l++] = ii;

            for(j = i + 1; j < nvtxs; j++)
            {
                iii = keys[j].val;

                if(keys[i].key != keys[j].key
                   || xadj[ii + 1] - xadj[ii] != xadj[iii + 1] - xadj[iii])
                    break; /* Break if keys or degrees are different */

                if(map[iii] == -1)
                { /* Do a comparison if iii has not been mapped */
                    for(jj = xadj[iii]; jj < xadj[iii + 1]; jj++)
                    {
                        if(mark[adjncy[jj]] != i)
                            break;
                    }

                    if(jj == xadj[iii + 1])
                    { /* Identical adjacency structure */
                        map[iii]  = cnvtxs;
                        cind[l++] = iii;
                    }
                }
            }

            cptr[++cnvtxs] = l;
        }
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_INFO,
          printf("  Compression: reduction in # of vertices: %" PRIDX ".\n", nvtxs - cnvtxs));


    if(cnvtxs < COMPRESSION_FRACTION * nvtxs)
    {
        /* Sufficient compression is possible, so go ahead and create the
       compressed graph */

        graph = CreateGraph();

        cnedges = 0;
        for(i = 0; i < cnvtxs; i++)
        {
            ii = cind[cptr[i]];
            cnedges += xadj[ii + 1] - xadj[ii];
        }

        /* Allocate memory for the compressed graph */
        cxadj = graph->xadj = imalloc(cnvtxs + 1, "CompressGraph: xadj");
        cvwgt = graph->vwgt = ismalloc(cnvtxs, 0, "CompressGraph: vwgt");
        cadjncy = graph->adjncy = imalloc(cnedges, "CompressGraph: adjncy");
        graph->adjwgt           = ismalloc(cnedges, 1, "CompressGraph: adjwgt");

        /* Now go and compress the graph */
        iset(nvtxs, -1, mark);
        l = cxadj[0] = 0;
        for(i = 0; i < cnvtxs; i++)
        {
            mark[i] = i; /* Remove any dioganal entries in the compressed graph */
            for(j = cptr[i]; j < cptr[i + 1]; j++)
            {
                ii = cind[j];

                /* accumulate the vertex weights of the constituent vertices */
                cvwgt[i] += (vwgt == NULL ? 1 : vwgt[ii]);

                /* generate the combined adjacency list */
                for(jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                {
                    k = map[adjncy[jj]];
                    if(mark[k] != i)
                    {
                        mark[k]      = i;
                        cadjncy[l++] = k;
                    }
                }
            }
            cxadj[i + 1] = l;
        }

        graph->nvtxs  = cnvtxs;
        graph->nedges = l;
        graph->ncon   = 1;

        SetupGraph_tvwgt(graph);
        SetupGraph_label(graph);
    }

    gk_free((void**)&keys, &map, &mark, LTERM);

    return graph;
}


/*************************************************************************/
/*! This function prunes all the vertices in a graph with degree greater
    than factor*average.

    \returns the number of vertices that were prunned.
*/
/*************************************************************************/
graph_t* PruneGraph(ctrl_t* ctrl, idx_t nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* iperm, real_t factor)
{
    idx_t    i, j, k, l, nlarge, pnvtxs, pnedges;
    idx_t *  pxadj, *padjncy, *padjwgt, *pvwgt;
    idx_t*   perm;
    graph_t* graph = NULL;

    perm = imalloc(nvtxs, "PruneGraph: perm");

    factor = factor * xadj[nvtxs] / nvtxs;

    pnvtxs = pnedges = nlarge = 0;
    for(i = 0; i < nvtxs; i++)
    {
        if(xadj[i + 1] - xadj[i] < factor)
        {
            perm[i]         = pnvtxs;
            iperm[pnvtxs++] = i;
            pnedges += xadj[i + 1] - xadj[i];
        }
        else
        {
            perm[i]               = nvtxs - ++nlarge;
            iperm[nvtxs - nlarge] = i;
        }
    }

    IFSET(ctrl->dbglvl,
          METIS_DBG_INFO,
          printf("  Pruned %" PRIDX " of %" PRIDX " vertices.\n", nlarge, nvtxs));


    if(nlarge > 0 && nlarge < nvtxs)
    {
        /* Prunning is possible, so go ahead and create the prunned graph */
        graph = CreateGraph();

        /* Allocate memory for the prunned graph*/
        pxadj = graph->xadj = imalloc(pnvtxs + 1, "PruneGraph: xadj");
        pvwgt = graph->vwgt = imalloc(pnvtxs, "PruneGraph: vwgt");
        padjncy = graph->adjncy = imalloc(pnedges, "PruneGraph: adjncy");
        graph->adjwgt           = ismalloc(pnedges, 1, "PruneGraph: adjwgt");

        pxadj[0] = pnedges = l = 0;
        for(i = 0; i < nvtxs; i++)
        {
            if(xadj[i + 1] - xadj[i] < factor)
            {
                pvwgt[l] = (vwgt == NULL ? 1 : vwgt[i]);

                for(j = xadj[i]; j < xadj[i + 1]; j++)
                {
                    k = perm[adjncy[j]];
                    if(k < pnvtxs)
                        padjncy[pnedges++] = k;
                }
                pxadj[++l] = pnedges;
            }
        }

        graph->nvtxs  = pnvtxs;
        graph->nedges = pnedges;
        graph->ncon   = 1;

        SetupGraph_tvwgt(graph);
        SetupGraph_label(graph);
    }
    else if(nlarge > 0 && nlarge == nvtxs)
    {
        IFSET(ctrl->dbglvl,
              METIS_DBG_INFO,
              printf("  Pruning is ignored as it removes all vertices.\n"));
        nlarge = 0;
    }


    gk_free((void**)&perm, LTERM);

    return graph;
}

/************************ mmd.c ************************/
/*
 * mmd.c
 *
 * **************************************************************
 * The following C function was developed from a FORTRAN subroutine
 * in SPARSPAK written by Eleanor Chu, Alan George, Joseph Liu
 * and Esmond Ng.
 *
 * The FORTRAN-to-C transformation and modifications such as dynamic
 * memory allocation and deallocation were performed by Chunguang
 * Sun.
 * **************************************************************
 *
 * Taken from SMMS, George 12/13/94
 *
 * The meaning of invperm, and perm vectors is different from that
 * in genqmd_ of SparsPak
 *
 * $Id: mmd.c 22385 2019-06-03 22:08:48Z karypis $
 */


/*************************************************************************
*  genmmd  -- multiple minimum external degree
*  purpose -- this routine implements the minimum degree
*     algorithm. it makes use of the implicit representation
*     of elimination graphs by quotient graphs, and the notion
*     of indistinguishable nodes. It also implements the modifications
*     by multiple elimination and minimum external degree.
*     Caution -- the adjacency vector adjncy will be destroyed.
*  Input parameters --
*     neqns -- number of equations.
*     (xadj, adjncy) -- the adjacency structure.
*     delta  -- tolerance value for multiple elimination.
*     maxint -- maximum machine representable (short) integer
*               (any smaller estimate will do) for marking nodes.
*  Output parameters --
*     perm -- the minimum degree ordering.
*     invp -- the inverse of perm.
*     *ncsub -- an upper bound on the number of nonzero subscripts
*               for the compressed storage scheme.
*  Working parameters --
*     head -- vector for head of degree lists.
*     invp  -- used temporarily for degree forward link.
*     perm  -- used temporarily for degree backward link.
*     qsize -- vector for size of supernodes.
*     list -- vector for temporary linked lists.
*     marker -- a temporary marker vector.
*  Subroutines used -- mmdelm, mmdint, mmdnum, mmdupd.
**************************************************************************/
void genmmd(idx_t  neqns,
            idx_t* xadj,
            idx_t* adjncy,
            idx_t* invp,
            idx_t* perm,
            idx_t  delta,
            idx_t* head,
            idx_t* qsize,
            idx_t* list,
            idx_t* marker,
            idx_t  maxint,
            idx_t* ncsub)
{
    idx_t ehead, i, mdeg, mdlmt, mdeg_node, nextmd, num, tag;

    if(neqns <= 0)
        return;

    /* adjust from C to Fortran */
    xadj--;
    adjncy--;
    invp--;
    perm--;
    head--;
    qsize--;
    list--;
    marker--;

    /* initialization for the minimum degree algorithm */
    *ncsub = 0;
    mmdint(neqns, xadj, adjncy, head, invp, perm, qsize, list, marker);

    /* 'num' counts the number of ordered nodes plus 1 */
    num = 1;

    /* eliminate all isolated nodes */
    nextmd = head[1];
    while(nextmd > 0)
    {
        mdeg_node         = nextmd;
        nextmd            = invp[mdeg_node];
        marker[mdeg_node] = maxint;
        invp[mdeg_node]   = -num;
        num++;
    }

    /* search for node of the minimum degree. 'mdeg' is the current */
    /* minimum degree; 'tag' is used to facilitate marking nodes.   */
    if(num > neqns)
        goto n1000;
    tag     = 1;
    head[1] = 0;
    mdeg    = 2;

    /* infinite loop here */
    while(1)
    {
        while(head[mdeg] <= 0)
            mdeg++;

        /* use value of 'delta' to set up 'mdlmt', which governs */
        /* when a degree update is to be performed.              */
        //mdlmt = mdeg + delta;
        // the need for gk_min() was identified by jsf67
        mdlmt = gk_min(neqns, mdeg + delta);
        ehead = 0;

    n500:
        mdeg_node = head[mdeg];
        while(mdeg_node <= 0)
        {
            mdeg++;

            if(mdeg > mdlmt)
                goto n900;
            mdeg_node = head[mdeg];
        };

        /* remove 'mdeg_node' from the degree structure */
        nextmd     = invp[mdeg_node];
        head[mdeg] = nextmd;
        if(nextmd > 0)
            perm[nextmd] = -mdeg;
        invp[mdeg_node] = -num;
        *ncsub += mdeg + qsize[mdeg_node] - 2;
        if((num + qsize[mdeg_node]) > neqns)
            goto n1000;

        /*  eliminate 'mdeg_node' and perform quotient graph */
        /*  transformation. reset 'tag' value if necessary.    */
        tag++;
        if(tag >= maxint)
        {
            tag = 1;
            for(i = 1; i <= neqns; i++)
                if(marker[i] < maxint)
                    marker[i] = 0;
        };

        mmdelm(mdeg_node, xadj, adjncy, head, invp, perm, qsize, list, marker, maxint, tag);

        num += qsize[mdeg_node];
        list[mdeg_node] = ehead;
        ehead           = mdeg_node;
        if(delta >= 0)
            goto n500;

    n900:
        /* update degrees of the nodes involved in the  */
        /* minimum degree nodes elimination.            */
        if(num > neqns)
            goto n1000;
        mmdupd(ehead, neqns, xadj, adjncy, delta, &mdeg, head, invp, perm, qsize, list, marker, maxint, &tag);
    }; /* end of -- while ( 1 ) -- */

n1000:
    mmdnum(neqns, perm, invp, qsize);

    /* Adjust from Fortran back to C*/
    xadj++;
    adjncy++;
    invp++;
    perm++;
    head++;
    qsize++;
    list++;
    marker++;
}


/**************************************************************************
*           mmdelm ...... multiple minimum degree elimination
* Purpose -- This routine eliminates the node mdeg_node of minimum degree
*     from the adjacency structure, which is stored in the quotient
*     graph format. It also transforms the quotient graph representation
*     of the elimination graph.
* Input parameters --
*     mdeg_node -- node of minimum degree.
*     maxint -- estimate of maximum representable (short) integer.
*     tag    -- tag value.
* Updated parameters --
*     (xadj, adjncy) -- updated adjacency structure.
*     (head, forward, backward) -- degree doubly linked structure.
*     qsize -- size of supernode.
*     marker -- marker vector.
*     list -- temporary linked list of eliminated nabors.
***************************************************************************/
void mmdelm(idx_t  mdeg_node,
            idx_t* xadj,
            idx_t* adjncy,
            idx_t* head,
            idx_t* forward,
            idx_t* backward,
            idx_t* qsize,
            idx_t* list,
            idx_t* marker,
            idx_t  maxint,
            idx_t  tag)
{
    idx_t element, i, istop, istart, j, jstop, jstart, link, nabor, node, npv,
        nqnbrs, nxnode, pvnode, rlmt, rloc, rnode, xqnbr;

    /* find the reachable set of 'mdeg_node' and */
    /* place it in the data structure.           */
    marker[mdeg_node] = tag;
    istart            = xadj[mdeg_node];
    istop             = xadj[mdeg_node + 1] - 1;

    /* 'element' points to the beginning of the list of  */
    /* eliminated nabors of 'mdeg_node', and 'rloc' gives the */
    /* storage location for the next reachable node.   */
    element = 0;
    rloc    = istart;
    rlmt    = istop;
    for(i = istart; i <= istop; i++)
    {
        nabor = adjncy[i];
        if(nabor == 0)
            break;
        if(marker[nabor] < tag)
        {
            marker[nabor] = tag;
            if(forward[nabor] < 0)
            {
                list[nabor] = element;
                element     = nabor;
            }
            else
            {
                adjncy[rloc] = nabor;
                rloc++;
            };
        }; /* end of -- if -- */
    }; /* end of -- for -- */

    /* merge with reachable nodes from generalized elements. */
    while(element > 0)
    {
        adjncy[rlmt] = -element;
        link         = element;

    n400:
        jstart = xadj[link];
        jstop  = xadj[link + 1] - 1;
        for(j = jstart; j <= jstop; j++)
        {
            node = adjncy[j];
            link = -node;
            if(node < 0)
                goto n400;
            if(node == 0)
                break;
            if((marker[node] < tag) && (forward[node] >= 0))
            {
                marker[node] = tag;
                /*use storage from eliminated nodes if necessary.*/
                while(rloc >= rlmt)
                {
                    link = -adjncy[rlmt];
                    rloc = xadj[link];
                    rlmt = xadj[link + 1] - 1;
                };
                adjncy[rloc] = node;
                rloc++;
            };
        }; /* end of -- for ( j = jstart; -- */
        element = list[element];
    }; /* end of -- while ( element > 0 ) -- */
    if(rloc <= rlmt)
        adjncy[rloc] = 0;
    /* for each node in the reachable set, do the following. */
    link = mdeg_node;

n1100:
    istart = xadj[link];
    istop  = xadj[link + 1] - 1;
    for(i = istart; i <= istop; i++)
    {
        rnode = adjncy[i];
        link  = -rnode;
        if(rnode < 0)
            goto n1100;
        if(rnode == 0)
            return;

        /* 'rnode' is in the degree list structure. */
        pvnode = backward[rnode];
        if((pvnode != 0) && (pvnode != (-maxint)))
        {
            /* then remove 'rnode' from the structure. */
            nxnode = forward[rnode];
            if(nxnode > 0)
                backward[nxnode] = pvnode;
            if(pvnode > 0)
                forward[pvnode] = nxnode;
            npv = -pvnode;
            if(pvnode < 0)
                head[npv] = nxnode;
        };

        /* purge inactive quotient nabors of 'rnode'. */
        jstart = xadj[rnode];
        jstop  = xadj[rnode + 1] - 1;
        xqnbr  = jstart;
        for(j = jstart; j <= jstop; j++)
        {
            nabor = adjncy[j];
            if(nabor == 0)
                break;
            if(marker[nabor] < tag)
            {
                adjncy[xqnbr] = nabor;
                xqnbr++;
            };
        };

        /* no active nabor after the purging. */
        nqnbrs = xqnbr - jstart;
        if(nqnbrs <= 0)
        {
            /* merge 'rnode' with 'mdeg_node'. */
            qsize[mdeg_node] += qsize[rnode];
            qsize[rnode]    = 0;
            marker[rnode]   = maxint;
            forward[rnode]  = -mdeg_node;
            backward[rnode] = -maxint;
        }
        else
        {
            /* flag 'rnode' for degree update, and  */
            /* add 'mdeg_node' as a nabor of 'rnode'.      */
            forward[rnode]  = nqnbrs + 1;
            backward[rnode] = 0;
            adjncy[xqnbr]   = mdeg_node;
            xqnbr++;
            if(xqnbr <= jstop)
                adjncy[xqnbr] = 0;
        };
    }; /* end of -- for ( i = istart; -- */
    return;
}


/***************************************************************************
*    mmdint ---- mult minimum degree initialization
*    purpose -- this routine performs initialization for the
*       multiple elimination version of the minimum degree algorithm.
*    input parameters --
*       neqns  -- number of equations.
*       (xadj, adjncy) -- adjacency structure.
*    output parameters --
*       (head, dfrow, backward) -- degree doubly linked structure.
*       qsize -- size of supernode ( initialized to one).
*       list -- linked list.
*       marker -- marker vector.
****************************************************************************/
idx_t mmdint(idx_t  neqns,
             idx_t* xadj,
             idx_t* adjncy,
             idx_t* head,
             idx_t* forward,
             idx_t* backward,
             idx_t* qsize,
             idx_t* list,
             idx_t* marker)
{
    idx_t fnode, ndeg, node;

    for(node = 1; node <= neqns; node++)
    {
        head[node]   = 0;
        qsize[node]  = 1;
        marker[node] = 0;
        list[node]   = 0;
    };

    /* initialize the degree doubly linked lists. */
    for(node = 1; node <= neqns; node++)
    {
        ndeg          = xadj[node + 1] - xadj[node] + 1;
        fnode         = head[ndeg];
        forward[node] = fnode;
        head[ndeg]    = node;
        if(fnode > 0)
            backward[fnode] = node;
        backward[node] = -ndeg;
    };

    return 0;
}


/****************************************************************************
* mmdnum --- multi minimum degree numbering
* purpose -- this routine performs the final step in producing
*    the permutation and inverse permutation vectors in the
*    multiple elimination version of the minimum degree
*    ordering algorithm.
* input parameters --
*     neqns -- number of equations.
*     qsize -- size of supernodes at elimination.
* updated parameters --
*     invp -- inverse permutation vector. on input,
*             if qsize[node] = 0, then node has been merged
*             into the node -invp[node]; otherwise,
*            -invp[node] is its inverse labelling.
* output parameters --
*     perm -- the permutation vector.
****************************************************************************/
void mmdnum(idx_t neqns, idx_t* perm, idx_t* invp, idx_t* qsize)
{
    idx_t father, nextf, node, nqsize, num, root;

    for(node = 1; node <= neqns; node++)
    {
        nqsize = qsize[node];
        if(nqsize <= 0)
            perm[node] = invp[node];
        if(nqsize > 0)
            perm[node] = -invp[node];
    };

    /* for each node which has been merged, do the following. */
    for(node = 1; node <= neqns; node++)
    {
        if(perm[node] <= 0)
        {

            /* trace the merged tree until one which has not */
            /* been merged, call it root.                    */
            father = node;
            while(perm[father] <= 0)
                father = -perm[father];

            /* number node after root. */
            root       = father;
            num        = perm[root] + 1;
            invp[node] = -num;
            perm[root] = num;

            /* shorten the merged tree. */
            father = node;
            nextf  = -perm[father];
            while(nextf > 0)
            {
                perm[father] = -root;
                father       = nextf;
                nextf        = -perm[father];
            };
        }; /* end of -- if ( perm[node] <= 0 ) -- */
    }; /* end of -- for ( node = 1; -- */

    /* ready to compute perm. */
    for(node = 1; node <= neqns; node++)
    {
        num        = -invp[node];
        invp[node] = num;
        perm[num]  = node;
    };
    return;
}


/****************************************************************************
* mmdupd ---- multiple minimum degree update
* purpose -- this routine updates the degrees of nodes after a
*            multiple elimination step.
* input parameters --
*    ehead -- the beginning of the list of eliminated nodes
*             (i.e., newly formed elements).
*    neqns -- number of equations.
*    (xadj, adjncy) -- adjacency structure.
*    delta -- tolerance value for multiple elimination.
*    maxint -- maximum machine representable (short) integer.
* updated parameters --
*    mdeg -- new minimum degree after degree update.
*    (head, forward, backward) -- degree doubly linked structure.
*    qsize -- size of supernode.
*    list -- marker vector for degree update.
*    *tag   -- tag value.
****************************************************************************/
void mmdupd(idx_t  ehead,
            idx_t  neqns,
            idx_t* xadj,
            idx_t* adjncy,
            idx_t  delta,
            idx_t* mdeg,
            idx_t* head,
            idx_t* forward,
            idx_t* backward,
            idx_t* qsize,
            idx_t* list,
            idx_t* marker,
            idx_t  maxint,
            idx_t* tag)
{
    idx_t deg, deg0, element, enode, fnode, i, iq2, istop, istart, j, jstop,
        jstart, link, mdeg0, mtag, nabor, node, q2head, qxhead;

    mdeg0   = *mdeg + delta;
    element = ehead;

n100:
    if(element <= 0)
        return;

    /* for each of the newly formed element, do the following. */
    /* reset tag value if necessary.                           */
    mtag = *tag + mdeg0;
    if(mtag >= maxint)
    {
        *tag = 1;
        for(i = 1; i <= neqns; i++)
            if(marker[i] < maxint)
                marker[i] = 0;
        mtag = *tag + mdeg0;
    };

    /* create two linked lists from nodes associated with 'element': */
    /* one with two nabors (q2head) in the adjacency structure, and the*/
    /* other with more than two nabors (qxhead). also compute 'deg0',*/
    /* number of nodes in this element.                              */
    q2head = 0;
    qxhead = 0;
    deg0   = 0;
    link   = element;

n400:
    istart = xadj[link];
    istop  = xadj[link + 1] - 1;
    for(i = istart; i <= istop; i++)
    {
        enode = adjncy[i];
        link  = -enode;
        if(enode < 0)
            goto n400;
        if(enode == 0)
            break;
        if(qsize[enode] != 0)
        {
            deg0 += qsize[enode];
            marker[enode] = mtag;

            /*'enode' requires a degree update*/
            if(backward[enode] == 0)
            {
                /* place either in qxhead or q2head list. */
                if(forward[enode] != 2)
                {
                    list[enode] = qxhead;
                    qxhead      = enode;
                }
                else
                {
                    list[enode] = q2head;
                    q2head      = enode;
                };
            };
        }; /* enf of -- if ( qsize[enode] != 0 ) -- */
    }; /* end of -- for ( i = istart; -- */

    /* for each node in q2 list, do the following. */
    enode = q2head;
    iq2   = 1;

n900:
    if(enode <= 0)
        goto n1500;
    if(backward[enode] != 0)
        goto n2200;
    (*tag)++;
    deg = deg0;

    /* identify the other adjacent element nabor. */
    istart = xadj[enode];
    nabor  = adjncy[istart];
    if(nabor == element)
        nabor = adjncy[istart + 1];
    link = nabor;
    if(forward[nabor] >= 0)
    {
        /* nabor is uneliminated, increase degree count. */
        deg += qsize[nabor];
        goto n2100;
    };

    /* the nabor is eliminated. for each node in the 2nd element */
    /* do the following.                                         */
n1000:
    istart = xadj[link];
    istop  = xadj[link + 1] - 1;
    for(i = istart; i <= istop; i++)
    {
        node = adjncy[i];
        link = -node;
        if(node != enode)
        {
            if(node < 0)
                goto n1000;
            if(node == 0)
                goto n2100;
            if(qsize[node] != 0)
            {
                if(marker[node] < *tag)
                {
                    /* 'node' is not yet considered. */
                    marker[node] = *tag;
                    deg += qsize[node];
                }
                else
                {
                    if(backward[node] == 0)
                    {
                        if(forward[node] == 2)
                        {
                            /* 'node' is indistinguishable from 'enode'.*/
                            /* merge them into a new supernode.         */
                            qsize[enode] += qsize[node];
                            qsize[node]    = 0;
                            marker[node]   = maxint;
                            forward[node]  = -enode;
                            backward[node] = -maxint;
                        }
                        else
                        {
                            /* 'node' is outmacthed by 'enode' */
                            if(backward[node] == 0)
                                backward[node] = -maxint;
                        };
                    }; /* end of -- if ( backward[node] == 0 ) -- */
                }; /* end of -- if ( marker[node] < *tag ) -- */
            }; /* end of -- if ( qsize[node] != 0 ) -- */
        }; /* end of -- if ( node != enode ) -- */
    }; /* end of -- for ( i = istart; -- */
    goto n2100;

n1500:
    /* for each 'enode' in the 'qx' list, do the following. */
    enode = qxhead;
    iq2   = 0;

n1600:
    if(enode <= 0)
        goto n2300;
    if(backward[enode] != 0)
        goto n2200;
    (*tag)++;
    deg = deg0;

    /*for each unmarked nabor of 'enode', do the following.*/
    istart = xadj[enode];
    istop  = xadj[enode + 1] - 1;
    for(i = istart; i <= istop; i++)
    {
        nabor = adjncy[i];
        if(nabor == 0)
            break;
        if(marker[nabor] < *tag)
        {
            marker[nabor] = *tag;
            link          = nabor;
            if(forward[nabor] >= 0)
                /*if uneliminated, include it in deg count.*/
                deg += qsize[nabor];
            else
            {
            n1700:
                /* if eliminated, include unmarked nodes in this*/
                /* element into the degree count.             */
                jstart = xadj[link];
                jstop  = xadj[link + 1] - 1;
                for(j = jstart; j <= jstop; j++)
                {
                    node = adjncy[j];
                    link = -node;
                    if(node < 0)
                        goto n1700;
                    if(node == 0)
                        break;
                    if(marker[node] < *tag)
                    {
                        marker[node] = *tag;
                        deg += qsize[node];
                    };
                }; /* end of -- for ( j = jstart; -- */
            }; /* end of -- if ( forward[nabor] >= 0 ) -- */
        }; /* end of -- if ( marker[nabor] < *tag ) -- */
    }; /* end of -- for ( i = istart; -- */

n2100:
    /* update external degree of 'enode' in degree structure, */
    /* and '*mdeg' if necessary.                     */
    deg             = deg - qsize[enode] + 1;
    fnode           = head[deg];
    forward[enode]  = fnode;
    backward[enode] = -deg;
    if(fnode > 0)
        backward[fnode] = enode;
    head[deg] = enode;
    if(deg < *mdeg)
        *mdeg = deg;

n2200:
    /* get next enode in current element. */
    enode = list[enode];
    if(iq2 == 1)
        goto n900;
    goto n1600;

n2300:
    /* get next element in the list. */
    *tag    = mtag;
    element = list[element];
    goto n100;
}

/************************ ometis.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * ometis.c
 *
 * This file contains the top level routines for the multilevel recursive
 * bisection algorithm PMETIS.
 *
 * Started 7/24/97
 * George
 *
 * $Id: ometis.c 10513 2011-07-07 22:06:03Z karypis $
 *
 */


/*************************************************************************/
/*! This function is the entry point for the multilevel nested dissection
    ordering code. At each bisection, a node-separator is computed using
    a node-based refinement approach.

    \param nvtxs is the number of vertices in the graph.
    \param xadj is of length nvtxs+1 marking the start of the adjancy
           list of each vertex in adjncy.
    \param adjncy stores the adjacency lists of the vertices. The adjnacy
           list of a vertex should not contain the vertex itself.
    \param vwgt is an array of size nvtxs storing the weight of each
           vertex. If vwgt is NULL, then the vertices are considered
           to have unit weight.
    \param numflag is either 0 or 1 indicating that the numbering of
           the vertices starts from 0 or 1, respectively.
    \param options is an array of size METIS_NOPTIONS used to pass
           various options impacting the of the algorithm. A NULL
           value indicates use of default options.
    \param perm is an array of size nvtxs such that if A and A' are
           the original and permuted matrices, then A'[i] = A[perm[i]].
    \param iperm is an array of size nvtxs such that if A and A' are
           the original and permuted matrices, then A[i] = A'[iperm[i]].
*/
/*************************************************************************/
int METIS_NodeND(idx_t* nvtxs, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* options, idx_t* perm, idx_t* iperm)
{
    int      sigrval = 0, renumber = 0;
    idx_t    i, ii, j, l, nnvtxs = 0;
    graph_t* graph = NULL;
    ctrl_t*  ctrl;
    idx_t *  cptr, *cind, *piperm;
    int      numflag = 0;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;


    /* set up the run time parameters */
    ctrl = SetupCtrl(METIS_OP_OMETIS, options, 1, 3, NULL, NULL);
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

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, InitTimers(ctrl));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->TotalTmr));

    /* prune the dense columns */
    if(ctrl->pfactor > 0.0)
    {
        piperm = imalloc(*nvtxs, "OMETIS: piperm");

        graph = PruneGraph(ctrl, *nvtxs, xadj, adjncy, vwgt, piperm, ctrl->pfactor);
        if(graph == NULL)
        {
            /* if there was no prunning, cleanup the pfactor */
            gk_free((void**)&piperm, LTERM);
            ctrl->pfactor = 0.0;
        }
        else
        {
            nnvtxs         = graph->nvtxs;
            ctrl->compress = 0; /* disable compression if prunning took place */
        }
    }

    /* compress the graph; note that compression only happens if not prunning
     has taken place. */
    if(ctrl->compress)
    {
        cptr = imalloc(*nvtxs + 1, "OMETIS: cptr");
        cind = imalloc(*nvtxs, "OMETIS: cind");

        graph = CompressGraph(ctrl, *nvtxs, xadj, adjncy, vwgt, cptr, cind);
        if(graph == NULL)
        {
            /* if there was no compression, cleanup the compress flag */
            gk_free((void**)&cptr, &cind, LTERM);
            ctrl->compress = 0;
        }
        else
        {
            nnvtxs        = graph->nvtxs;
            ctrl->cfactor = 1.0 * (*nvtxs) / nnvtxs;
            if(ctrl->cfactor > 1.5 && ctrl->nseps == 1)
                ctrl->nseps = 2;
            //ctrl->nseps = (idx_t)(ctrl->cfactor*ctrl->nseps);
        }
    }

    /* if no prunning and no compression, setup the graph in the normal way. */
    if(ctrl->pfactor == 0.0 && ctrl->compress == 0)
        graph = SetupGraph(ctrl, *nvtxs, 1, xadj, adjncy, vwgt, NULL, NULL);

    ASSERT(CheckGraph(graph, ctrl->numflag, 1));

    /* allocate workspace memory */
    AllocateWorkSpace(ctrl, graph);

    /* do the nested dissection ordering  */
    if(ctrl->ccorder)
        MlevelNestedDissectionCC(ctrl, graph, iperm, graph->nvtxs);
    else
        MlevelNestedDissection(ctrl, graph, iperm, graph->nvtxs);


    if(ctrl->pfactor > 0.0)
    {                               /* Order any prunned vertices */
        icopy(nnvtxs, iperm, perm); /* Use perm as an auxiliary array */
        for(i = 0; i < nnvtxs; i++)
            iperm[piperm[i]] = perm[i];
        for(i = nnvtxs; i < *nvtxs; i++)
            iperm[piperm[i]] = i;

        gk_free((void**)&piperm, LTERM);
    }
    else if(ctrl->compress)
    { /* Uncompress the ordering */
        /* construct perm from iperm */
        for(i = 0; i < nnvtxs; i++)
            perm[iperm[i]] = i;
        for(l = ii = 0; ii < nnvtxs; ii++)
        {
            i = perm[ii];
            for(j = cptr[i]; j < cptr[i + 1]; j++)
                iperm[cind[j]] = l++;
        }

        gk_free((void**)&cptr, &cind, LTERM);
    }

    for(i = 0; i < *nvtxs; i++)
        perm[iperm[i]] = i;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->TotalTmr));
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, PrintTimers(ctrl));

    /* clean up */
    FreeCtrl(&ctrl);

SIGTHROW:
    /* if required, change the numbering back to 1 */
    if(renumber)
        Change2FNumberingOrder(*nvtxs, xadj, adjncy, perm, iperm);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    return metis_rcode(sigrval);
}


/*************************************************************************/
/*! This is the driver for the recursive tri-section of a graph into the
    left, separator, and right partitions. The graphs correspond to the
    left and right parts are further tri-sected in a recursive fashion.
    The nodes in the separator are ordered at the end of the left & right
    nodes.
 */
/*************************************************************************/
void MlevelNestedDissection(ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx)
{
    idx_t    i, j, nvtxs, nbnd;
    idx_t *  label, *bndind;
    graph_t *lgraph, *rgraph;

    nvtxs = graph->nvtxs;

    MlevelNodeBisectionMultiple(ctrl, graph);

    IFSET(ctrl->dbglvl,
          METIS_DBG_SEPINFO,
          printf("Nvtxs: %6" PRIDX ", [%6" PRIDX " %6" PRIDX " %6" PRIDX "]\n",
                 graph->nvtxs,
                 graph->pwgts[0],
                 graph->pwgts[1],
                 graph->pwgts[2]));


    /* Order the nodes in the separator */
    nbnd   = graph->nbnd;
    bndind = graph->bndind;
    label  = graph->label;
    for(i = 0; i < nbnd; i++)
        order[label[bndind[i]]] = --lastvtx;

    SplitGraphOrder(ctrl, graph, &lgraph, &rgraph);

    /* Free the memory of the top level graph */
    FreeGraph(&graph);

    /* Recurse on lgraph first, as its lastvtx depends on rgraph->nvtxs, which
     will not be defined upon return from MlevelNestedDissection. */
    if(lgraph->nvtxs > MMDSWITCH && lgraph->nedges > 0)
        MlevelNestedDissection(ctrl, lgraph, order, lastvtx - rgraph->nvtxs);
    else
    {
        MMDOrder(ctrl, lgraph, order, lastvtx - rgraph->nvtxs);
        FreeGraph(&lgraph);
    }
    if(rgraph->nvtxs > MMDSWITCH && rgraph->nedges > 0)
        MlevelNestedDissection(ctrl, rgraph, order, lastvtx);
    else
    {
        MMDOrder(ctrl, rgraph, order, lastvtx);
        FreeGraph(&rgraph);
    }
}


/*************************************************************************/
/*! This routine is similar to its non 'CC' counterpart. The difference is
    that after each tri-section, the connected components of the original
    graph that result after removing the separator vertises are ordered
    independently (i.e., this may lead to more than just the left and
    the right subgraphs).
*/
/*************************************************************************/
void MlevelNestedDissectionCC(ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx)
{
    idx_t     i, j, nvtxs, nbnd, ncmps, rnvtxs, snvtxs;
    idx_t *   label, *bndind;
    idx_t *   cptr, *cind;
    graph_t** sgraphs;

    nvtxs = graph->nvtxs;

    MlevelNodeBisectionMultiple(ctrl, graph);

    IFSET(ctrl->dbglvl,
          METIS_DBG_SEPINFO,
          printf("Nvtxs: %6" PRIDX ", [%6" PRIDX " %6" PRIDX " %6" PRIDX "]\n",
                 graph->nvtxs,
                 graph->pwgts[0],
                 graph->pwgts[1],
                 graph->pwgts[2]));

    /* Order the nodes in the separator */
    nbnd   = graph->nbnd;
    bndind = graph->bndind;
    label  = graph->label;
    for(i = 0; i < nbnd; i++)
        order[label[bndind[i]]] = --lastvtx;

    WCOREPUSH;
    cptr  = iwspacemalloc(ctrl, nvtxs + 1);
    cind  = iwspacemalloc(ctrl, nvtxs);
    ncmps = FindSepInducedComponents(ctrl, graph, cptr, cind);

    if(ctrl->dbglvl & METIS_DBG_INFO)
    {
        if(ncmps > 2)
            printf("  Bisection resulted in %" PRIDX " connected components\n", ncmps);
    }

    sgraphs = SplitGraphOrderCC(ctrl, graph, ncmps, cptr, cind);

    WCOREPOP;

    /* Free the memory of the top level graph */
    FreeGraph(&graph);

    /* Go and process the subgraphs */
    for(rnvtxs = i = 0; i < ncmps; i++)
    {
        /* Save the number of vertices in sgraphs[i] because sgraphs[i] is freed
       inside MlevelNestedDissectionCC, and as such it will be undefined. */
        snvtxs = sgraphs[i]->nvtxs;

        if(sgraphs[i]->nvtxs > MMDSWITCH && sgraphs[i]->nedges > 0)
        {
            MlevelNestedDissectionCC(ctrl, sgraphs[i], order, lastvtx - rnvtxs);
        }
        else
        {
            MMDOrder(ctrl, sgraphs[i], order, lastvtx - rnvtxs);
            FreeGraph(&sgraphs[i]);
        }
        rnvtxs += snvtxs;
    }

    gk_free((void**)&sgraphs, LTERM);
}


/*************************************************************************/
/*! This function performs multilevel node bisection (i.e., tri-section).
    It performs multiple bisections and selects the best. */
/*************************************************************************/
void MlevelNodeBisectionMultiple(ctrl_t* ctrl, graph_t* graph)
{
    idx_t  i, mincut;
    idx_t* bestwhere;

    /* if the graph is small, just find a single vertex separator */
    if(ctrl->nseps == 1 || graph->nvtxs < (ctrl->compress ? 1000 : 2000))
    {
        MlevelNodeBisectionL2(ctrl, graph, LARGENIPARTS);
        return;
    }

    WCOREPUSH;

    bestwhere = iwspacemalloc(ctrl, graph->nvtxs);

    mincut = graph->tvwgt[0];
    for(i = 0; i < ctrl->nseps; i++)
    {
        MlevelNodeBisectionL2(ctrl, graph, LARGENIPARTS);

        if(i == 0 || graph->mincut < mincut)
        {
            mincut = graph->mincut;
            if(i < ctrl->nseps - 1)
                icopy(graph->nvtxs, graph->where, bestwhere);
        }

        if(mincut == 0)
            break;

        if(i < ctrl->nseps - 1)
            FreeRData(graph);
    }

    if(mincut != graph->mincut)
    {
        icopy(graph->nvtxs, bestwhere, graph->where);
        Compute2WayNodePartitionParams(ctrl, graph);
    }

    WCOREPOP;
}


/*************************************************************************/
/*! This function performs multilevel node bisection (i.e., tri-section).
    It performs multiple bisections and selects the best. */
/*************************************************************************/
void MlevelNodeBisectionL2(ctrl_t* ctrl, graph_t* graph, idx_t niparts)
{
    idx_t    i, mincut, nruns = 5;
    graph_t* cgraph;
    idx_t*   bestwhere;

    /* if the graph is small, just find a single vertex separator */
    if(graph->nvtxs < 5000)
    {
        MlevelNodeBisectionL1(ctrl, graph, niparts);
        return;
    }

    WCOREPUSH;

    ctrl->CoarsenTo = gk_max(100, graph->nvtxs / 30);

    cgraph = CoarsenGraphNlevels(ctrl, graph, 4);

    bestwhere = iwspacemalloc(ctrl, cgraph->nvtxs);

    mincut = graph->tvwgt[0];
    for(i = 0; i < nruns; i++)
    {
        MlevelNodeBisectionL1(ctrl, cgraph, 0.7 * niparts);

        if(i == 0 || cgraph->mincut < mincut)
        {
            mincut = cgraph->mincut;
            if(i < nruns - 1)
                icopy(cgraph->nvtxs, cgraph->where, bestwhere);
        }

        if(mincut == 0)
            break;

        if(i < nruns - 1)
            FreeRData(cgraph);
    }

    if(mincut != cgraph->mincut)
        icopy(cgraph->nvtxs, bestwhere, cgraph->where);

    WCOREPOP;

    Refine2WayNode(ctrl, graph, cgraph);
}


/*************************************************************************/
/*! The top-level routine of the actual multilevel node bisection */
/*************************************************************************/
void MlevelNodeBisectionL1(ctrl_t* ctrl, graph_t* graph, idx_t niparts)
{
    graph_t* cgraph;

    ctrl->CoarsenTo = graph->nvtxs / 8;
    if(ctrl->CoarsenTo > 100)
        ctrl->CoarsenTo = 100;
    else if(ctrl->CoarsenTo < 40)
        ctrl->CoarsenTo = 40;

    cgraph = CoarsenGraph(ctrl, graph);

    niparts = gk_max(1, (cgraph->nvtxs <= ctrl->CoarsenTo ? niparts / 2 : niparts));
    /*niparts = (cgraph->nvtxs <= ctrl->CoarsenTo ? SMALLNIPARTS : LARGENIPARTS);*/
    InitSeparator(ctrl, cgraph, niparts);

    Refine2WayNode(ctrl, graph, cgraph);
}


/*************************************************************************/
/*! This function takes a graph and a tri-section (left, right, separator)
    and splits it into two graphs.

    This function relies on the fact that adjwgt is all equal to 1.
*/
/*************************************************************************/
void SplitGraphOrder(ctrl_t* ctrl, graph_t* graph, graph_t** r_lgraph, graph_t** r_rgraph)
{
    idx_t    i, ii, j, k, l, istart, iend, mypart, nvtxs, snvtxs[3], snedges[3];
    idx_t *  xadj, *vwgt, *adjncy, *adjwgt, *label, *where, *bndptr, *bndind;
    idx_t *  sxadj[2], *svwgt[2], *sadjncy[2], *sadjwgt[2], *slabel[2];
    idx_t*   rename;
    idx_t*   auxadjncy;
    graph_t *lgraph, *rgraph;

    WCOREPUSH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->SplitTmr));

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    label  = graph->label;
    where  = graph->where;
    bndptr = graph->bndptr;
    bndind = graph->bndind;
    ASSERT(bndptr != NULL);

    rename = iwspacemalloc(ctrl, nvtxs);

    snvtxs[0] = snvtxs[1] = snvtxs[2] = snedges[0] = snedges[1] = snedges[2] = 0;
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

    /* Go and use bndptr to also mark the boundary nodes in the two partitions */
    for(ii = 0; ii < graph->nbnd; ii++)
    {
        i = bndind[ii];
        for(j = xadj[i]; j < xadj[i + 1]; j++)
            bndptr[adjncy[j]] = 1;
    }

    snvtxs[0] = snvtxs[1] = snedges[0] = snedges[1] = 0;
    sxadj[0][0] = sxadj[1][0] = 0;
    for(i = 0; i < nvtxs; i++)
    {
        if((mypart = where[i]) == 2)
            continue;

        istart = xadj[i];
        iend   = xadj[i + 1];
        if(bndptr[i] == -1)
        { /* This is an interior vertex */
            auxadjncy = sadjncy[mypart] + snedges[mypart] - istart;
            for(j = istart; j < iend; j++)
                auxadjncy[j] = adjncy[j];
            snedges[mypart] += iend - istart;
        }
        else
        {
            auxadjncy = sadjncy[mypart];
            l         = snedges[mypart];
            for(j = istart; j < iend; j++)
            {
                k = adjncy[j];
                if(where[k] == mypart)
                    auxadjncy[l++] = k;
            }
            snedges[mypart] = l;
        }

        svwgt[mypart][snvtxs[mypart]]   = vwgt[i];
        slabel[mypart][snvtxs[mypart]]  = label[i];
        sxadj[mypart][++snvtxs[mypart]] = snedges[mypart];
    }

    for(mypart = 0; mypart < 2; mypart++)
    {
        iend = snedges[mypart];
        iset(iend, 1, sadjwgt[mypart]);

        auxadjncy = sadjncy[mypart];
        for(i = 0; i < iend; i++)
            auxadjncy[i] = rename[auxadjncy[i]];
    }

    lgraph->nvtxs  = snvtxs[0];
    lgraph->nedges = snedges[0];
    rgraph->nvtxs  = snvtxs[1];
    rgraph->nedges = snedges[1];

    SetupGraph_tvwgt(lgraph);
    SetupGraph_tvwgt(rgraph);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->SplitTmr));

    *r_lgraph = lgraph;
    *r_rgraph = rgraph;

    WCOREPOP;
}


/*************************************************************************/
/*! This function takes a graph and generates a set of graphs, each of
    which is a connected component in the original graph.

    This function relies on the fact that adjwgt is all equal to 1.

    \param ctrl stores run state info.
    \param graph is the graph to be split.
    \param ncmps is the number of connected components.
    \param cptr is an array of size ncmps+1 that marks the start and end
           locations of the vertices in cind that make up the respective
           components (i.e., cptr, cind is in CSR format).
    \param cind is an array of size equal to the number of vertices in
           the original graph and stores the vertices that belong to each
           connected component.

    \returns an array of subgraphs corresponding to the extracted subgraphs.
*/
/*************************************************************************/
graph_t** SplitGraphOrderCC(ctrl_t* ctrl, graph_t* graph, idx_t ncmps, idx_t* cptr, idx_t* cind)
{
    idx_t     i, ii, iii, j, k, l, istart, iend, mypart, nvtxs, snvtxs, snedges;
    idx_t *   xadj, *vwgt, *adjncy, *adjwgt, *label, *where, *bndptr, *bndind;
    idx_t *   sxadj, *svwgt, *sadjncy, *sadjwgt, *slabel;
    idx_t*    rename;
    idx_t*    auxadjncy;
    graph_t** sgraphs;

    WCOREPUSH;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->SplitTmr));

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    vwgt   = graph->vwgt;
    adjncy = graph->adjncy;
    adjwgt = graph->adjwgt;
    label  = graph->label;
    where  = graph->where;
    bndptr = graph->bndptr;
    bndind = graph->bndind;
    ASSERT(bndptr != NULL);

    /* Go and use bndptr to also mark the boundary nodes in the two partitions */
    for(ii = 0; ii < graph->nbnd; ii++)
    {
        i = bndind[ii];
        for(j = xadj[i]; j < xadj[i + 1]; j++)
            bndptr[adjncy[j]] = 1;
    }

    rename = iwspacemalloc(ctrl, nvtxs);

    sgraphs = (graph_t**)gk_malloc(sizeof(graph_t*) * ncmps, "SplitGraphOrderCC: sgraphs");

    /* Go and split the graph a component at a time */
    for(iii = 0; iii < ncmps; iii++)
    {
        irandArrayPermute(
            cptr[iii + 1] - cptr[iii], cind + cptr[iii], cptr[iii + 1] - cptr[iii], 0);
        snvtxs = snedges = 0;
        for(j = cptr[iii]; j < cptr[iii + 1]; j++)
        {
            i         = cind[j];
            rename[i] = snvtxs++;
            snedges += xadj[i + 1] - xadj[i];
        }

        sgraphs[iii] = SetupSplitGraph(graph, snvtxs, snedges);

        sxadj   = sgraphs[iii]->xadj;
        svwgt   = sgraphs[iii]->vwgt;
        sadjncy = sgraphs[iii]->adjncy;
        sadjwgt = sgraphs[iii]->adjwgt;
        slabel  = sgraphs[iii]->label;

        snvtxs = snedges = sxadj[0] = 0;
        for(ii = cptr[iii]; ii < cptr[iii + 1]; ii++)
        {
            i = cind[ii];

            istart = xadj[i];
            iend   = xadj[i + 1];
            if(bndptr[i] == -1)
            { /* This is an interior vertex */
                auxadjncy = sadjncy + snedges - istart;
                for(j = istart; j < iend; j++)
                    auxadjncy[j] = adjncy[j];
                snedges += iend - istart;
            }
            else
            {
                l = snedges;
                for(j = istart; j < iend; j++)
                {
                    k = adjncy[j];
                    if(where[k] != 2)
                        sadjncy[l++] = k;
                }
                snedges = l;
            }

            svwgt[snvtxs]   = vwgt[i];
            slabel[snvtxs]  = label[i];
            sxadj[++snvtxs] = snedges;
        }

        iset(snedges, 1, sadjwgt);
        for(i = 0; i < snedges; i++)
            sadjncy[i] = rename[sadjncy[i]];

        sgraphs[iii]->nvtxs  = snvtxs;
        sgraphs[iii]->nedges = snedges;

        SetupGraph_tvwgt(sgraphs[iii]);
    }

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->SplitTmr));

    WCOREPOP;

    return sgraphs;
}


/*************************************************************************/
/*! This function uses MMD to order the graph. The vertices are numbered
    from lastvtx downwards. */
/*************************************************************************/
void MMDOrder(ctrl_t* ctrl, graph_t* graph, idx_t* order, idx_t lastvtx)
{
    idx_t  i, j, k, nvtxs, nofsub, firstvtx;
    idx_t *xadj, *adjncy, *label;
    idx_t *perm, *iperm, *head, *qsize, *list, *marker;

    WCOREPUSH;

    nvtxs  = graph->nvtxs;
    xadj   = graph->xadj;
    adjncy = graph->adjncy;

    /* Relabel the vertices so that it starts from 1 */
    k = xadj[nvtxs];
    for(i = 0; i < k; i++)
        adjncy[i]++;
    for(i = 0; i < nvtxs + 1; i++)
        xadj[i]++;

    perm   = iwspacemalloc(ctrl, nvtxs + 5);
    iperm  = iwspacemalloc(ctrl, nvtxs + 5);
    head   = iwspacemalloc(ctrl, nvtxs + 5);
    qsize  = iwspacemalloc(ctrl, nvtxs + 5);
    list   = iwspacemalloc(ctrl, nvtxs + 5);
    marker = iwspacemalloc(ctrl, nvtxs + 5);

    genmmd(nvtxs, xadj, adjncy, iperm, perm, 1, head, qsize, list, marker, IDX_MAX, &nofsub);

    label    = graph->label;
    firstvtx = lastvtx - nvtxs;
    for(i = 0; i < nvtxs; i++)
        order[label[i]] = firstvtx + iperm[i] - 1;

    /* Relabel the vertices so that it starts from 0 */
    for(i = 0; i < nvtxs + 1; i++)
        xadj[i]--;
    k = xadj[nvtxs];
    for(i = 0; i < k; i++)
        adjncy[i]--;

    WCOREPOP;
}
