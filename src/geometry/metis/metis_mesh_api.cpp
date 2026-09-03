/* Derived from METIS 5.2.1 and adapted for libuipc's C++ build.
 * See LICENSE-METIS in this directory.
 * The mesh conversion and partitioning algorithms remain source-equivalent.
 */

#include <metis.h>

/************************ mesh.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * mesh.c
 *
 * This file contains routines for converting 3D and 4D finite element
 * meshes into dual or nodal graphs
 *
 * Started 8/18/97
 * George
 *
 * $Id: mesh.c 13804 2013-03-04 23:49:08Z karypis $
 *
 */


/*****************************************************************************/
/*! This function creates a graph corresponding to the dual of a finite element
    mesh.

    \param ne is the number of elements in the mesh.
    \param nn is the number of nodes in the mesh.
    \param eptr is an array of size ne+1 used to mark the start and end
           locations in the nind array.
    \param eind is an array that stores for each element the set of node IDs
           (indices) that it is made off. The length of this array is equal
           to the total number of nodes over all the mesh elements.
    \param ncommon is the minimum number of nodes that two elements must share
           in order to be connected via an edge in the dual graph.
    \param numflag is either 0 or 1 indicating if the numbering of the nodes
           starts from 0 or 1, respectively. The same numbering is used for the
           returned graph as well.
    \param r_xadj indicates where the adjacency list of each vertex is stored
           in r_adjncy. The memory for this array is allocated by this routine.
           It can be freed by calling METIS_free().
    \param r_adjncy stores the adjacency list of each vertex in the generated
           dual graph. The memory for this array is allocated by this routine.
           It can be freed by calling METIS_free().

*/
/*****************************************************************************/
int METIS_MeshToDual(idx_t*  ne,
                     idx_t*  nn,
                     idx_t*  eptr,
                     idx_t*  eind,
                     idx_t*  ncommon,
                     idx_t*  numflag,
                     idx_t** r_xadj,
                     idx_t** r_adjncy)
{
    int sigrval = 0, renumber = 0;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;


    /* renumber the mesh */
    if(*numflag == 1)
    {
        ChangeMesh2CNumbering(*ne, eptr, eind);
        renumber = 1;
    }

    /* create dual graph */
    *r_xadj = *r_adjncy = NULL;
    CreateGraphDual(*ne, *nn, eptr, eind, *ncommon, r_xadj, r_adjncy);


SIGTHROW:
    if(renumber)
        ChangeMesh2FNumbering(*ne, eptr, eind, *ne, *r_xadj, *r_adjncy);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    if(sigrval != 0)
    {
        if(*r_xadj != NULL)
            free(*r_xadj);
        if(*r_adjncy != NULL)
            free(*r_adjncy);
        *r_xadj = *r_adjncy = NULL;
    }

    return metis_rcode(sigrval);
}


/*****************************************************************************/
/*! This function creates a graph corresponding to (almost) the nodal of a
    finite element mesh. In the nodal graph, each node is connected to the
    nodes corresponding to the union of nodes present in all the elements
    in which that node belongs.

    \param ne is the number of elements in the mesh.
    \param nn is the number of nodes in the mesh.
    \param eptr is an array of size ne+1 used to mark the start and end
           locations in the nind array.
    \param eind is an array that stores for each element the set of node IDs
           (indices) that it is made off. The length of this array is equal
           to the total number of nodes over all the mesh elements.
    \param numflag is either 0 or 1 indicating if the numbering of the nodes
           starts from 0 or 1, respectively. The same numbering is used for the
           returned graph as well.
    \param r_xadj indicates where the adjacency list of each vertex is stored
           in r_adjncy. The memory for this array is allocated by this routine.
           It can be freed by calling METIS_free().
    \param r_adjncy stores the adjacency list of each vertex in the generated
           dual graph. The memory for this array is allocated by this routine.
           It can be freed by calling METIS_free().

*/
/*****************************************************************************/
int METIS_MeshToNodal(
    idx_t* ne, idx_t* nn, idx_t* eptr, idx_t* eind, idx_t* numflag, idx_t** r_xadj, idx_t** r_adjncy)
{
    int sigrval = 0, renumber = 0;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;


    /* renumber the mesh */
    if(*numflag == 1)
    {
        ChangeMesh2CNumbering(*ne, eptr, eind);
        renumber = 1;
    }

    /* create nodal graph */
    *r_xadj = *r_adjncy = NULL;
    CreateGraphNodal(*ne, *nn, eptr, eind, r_xadj, r_adjncy);


SIGTHROW:
    if(renumber)
        ChangeMesh2FNumbering(*ne, eptr, eind, *nn, *r_xadj, *r_adjncy);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    if(sigrval != 0)
    {
        if(*r_xadj != NULL)
            free(*r_xadj);
        if(*r_adjncy != NULL)
            free(*r_adjncy);
        *r_xadj = *r_adjncy = NULL;
    }

    return metis_rcode(sigrval);
}


/*****************************************************************************/
/*! This function creates the dual of a finite element mesh */
/*****************************************************************************/
void CreateGraphDual(idx_t ne, idx_t nn, idx_t* eptr, idx_t* eind, idx_t ncommon, idx_t** r_xadj, idx_t** r_adjncy)
{
    idx_t  i, j, nnbrs;
    idx_t *nptr, *nind;
    idx_t *xadj, *adjncy;
    idx_t *marker, *nbrs;

    if(ncommon < 1)
    {
        printf("  Increased ncommon to 1, as it was initially %" PRIDX "\n", ncommon);
        ncommon = 1;
    }

    /* construct the node-element list first */
    nptr = ismalloc(nn + 1, 0, "CreateGraphDual: nptr");
    nind = imalloc(eptr[ne], "CreateGraphDual: nind");

    for(i = 0; i < ne; i++)
    {
        for(j = eptr[i]; j < eptr[i + 1]; j++)
            nptr[eind[j]]++;
    }
    MAKECSR(i, nn, nptr);

    for(i = 0; i < ne; i++)
    {
        for(j = eptr[i]; j < eptr[i + 1]; j++)
            nind[nptr[eind[j]]++] = i;
    }
    SHIFTCSR(i, nn, nptr);


    /* Allocate memory for xadj, since you know its size.
     These are done using standard malloc as they are returned
     to the calling function */
    if((xadj = (idx_t*)malloc((ne + 1) * sizeof(idx_t))) == NULL)
        gk_errexit(SIGMEM, "***Failed to allocate memory for xadj.\n");
    *r_xadj = xadj;
    iset(ne + 1, 0, xadj);

    /* allocate memory for working arrays used by FindCommonElements */
    marker = ismalloc(ne, 0, "CreateGraphDual: marker");
    nbrs   = imalloc(ne, "CreateGraphDual: nbrs");

    for(i = 0; i < ne; i++)
    {
        xadj[i] = FindCommonElements(
            i, eptr[i + 1] - eptr[i], eind + eptr[i], nptr, nind, eptr, ncommon, marker, nbrs);
    }
    MAKECSR(i, ne, xadj);

    /* Allocate memory for adjncy, since you now know its size.
     These are done using standard malloc as they are returned
     to the calling function */
    if((adjncy = (idx_t*)malloc(xadj[ne] * sizeof(idx_t))) == NULL)
    {
        free(xadj);
        *r_xadj = NULL;
        gk_errexit(SIGMEM, "***Failed to allocate memory for adjncy.\n");
    }
    *r_adjncy = adjncy;

    for(i = 0; i < ne; i++)
    {
        nnbrs = FindCommonElements(
            i, eptr[i + 1] - eptr[i], eind + eptr[i], nptr, nind, eptr, ncommon, marker, nbrs);
        for(j = 0; j < nnbrs; j++)
            adjncy[xadj[i]++] = nbrs[j];
    }
    SHIFTCSR(i, ne, xadj);

    gk_free((void**)&nptr, &nind, &marker, &nbrs, LTERM);
}


/*****************************************************************************/
/*! This function finds all elements that share at least ncommon nodes with
    the ``query'' element.
*/
/*****************************************************************************/
idx_t FindCommonElements(idx_t  qid,
                         idx_t  elen,
                         idx_t* eind,
                         idx_t* nptr,
                         idx_t* nind,
                         idx_t* eptr,
                         idx_t  ncommon,
                         idx_t* marker,
                         idx_t* nbrs)
{
    idx_t i, ii, j, jj, k, l, overlap;

    /* find all elements that share at least one node with qid */
    for(k = 0, i = 0; i < elen; i++)
    {
        j = eind[i];
        for(ii = nptr[j]; ii < nptr[j + 1]; ii++)
        {
            jj = nind[ii];

            if(marker[jj] == 0)
                nbrs[k++] = jj;
            marker[jj]++;
        }
    }

    /* put qid into the neighbor list (in case it is not there) so that it
     will be removed in the next step */
    if(marker[qid] == 0)
        nbrs[k++] = qid;
    marker[qid] = 0;

    /* compact the list to contain only those with at least ncommon nodes */
    for(j = 0, i = 0; i < k; i++)
    {
        overlap = marker[l = nbrs[i]];
        if(overlap >= ncommon || overlap >= elen - 1
           || overlap >= eptr[l + 1] - eptr[l] - 1)
            nbrs[j++] = l;
        marker[l] = 0;
    }

    return j;
}


/*****************************************************************************/
/*! This function creates the (almost) nodal of a finite element mesh */
/*****************************************************************************/
void CreateGraphNodal(idx_t ne, idx_t nn, idx_t* eptr, idx_t* eind, idx_t** r_xadj, idx_t** r_adjncy)
{
    idx_t  i, j, nnbrs;
    idx_t *nptr, *nind;
    idx_t *xadj, *adjncy;
    idx_t *marker, *nbrs;


    /* construct the node-element list first */
    nptr = ismalloc(nn + 1, 0, "CreateGraphNodal: nptr");
    nind = imalloc(eptr[ne], "CreateGraphNodal: nind");

    for(i = 0; i < ne; i++)
    {
        for(j = eptr[i]; j < eptr[i + 1]; j++)
            nptr[eind[j]]++;
    }
    MAKECSR(i, nn, nptr);

    for(i = 0; i < ne; i++)
    {
        for(j = eptr[i]; j < eptr[i + 1]; j++)
            nind[nptr[eind[j]]++] = i;
    }
    SHIFTCSR(i, nn, nptr);


    /* Allocate memory for xadj, since you know its size.
     These are done using standard malloc as they are returned
     to the calling function */
    if((xadj = (idx_t*)malloc((nn + 1) * sizeof(idx_t))) == NULL)
        gk_errexit(SIGMEM, "***Failed to allocate memory for xadj.\n");
    *r_xadj = xadj;
    iset(nn + 1, 0, xadj);

    /* allocate memory for working arrays used by FindCommonElements */
    marker = ismalloc(nn, 0, "CreateGraphNodal: marker");
    nbrs   = imalloc(nn, "CreateGraphNodal: nbrs");

    for(i = 0; i < nn; i++)
    {
        xadj[i] = FindCommonNodes(
            i, nptr[i + 1] - nptr[i], nind + nptr[i], eptr, eind, marker, nbrs);
    }
    MAKECSR(i, nn, xadj);

    /* Allocate memory for adjncy, since you now know its size.
     These are done using standard malloc as they are returned
     to the calling function */
    if((adjncy = (idx_t*)malloc(xadj[nn] * sizeof(idx_t))) == NULL)
    {
        free(xadj);
        *r_xadj = NULL;
        gk_errexit(SIGMEM, "***Failed to allocate memory for adjncy.\n");
    }
    *r_adjncy = adjncy;

    for(i = 0; i < nn; i++)
    {
        nnbrs = FindCommonNodes(i, nptr[i + 1] - nptr[i], nind + nptr[i], eptr, eind, marker, nbrs);
        for(j = 0; j < nnbrs; j++)
            adjncy[xadj[i]++] = nbrs[j];
    }
    SHIFTCSR(i, nn, xadj);

    gk_free((void**)&nptr, &nind, &marker, &nbrs, LTERM);
}


/*****************************************************************************/
/*! This function finds the union of nodes that are in the same elements with
    the ``query'' node.
*/
/*****************************************************************************/
idx_t FindCommonNodes(
    idx_t qid, idx_t nelmnts, idx_t* elmntids, idx_t* eptr, idx_t* eind, idx_t* marker, idx_t* nbrs)
{
    idx_t i, ii, j, jj, k;

    /* find all nodes that share at least one element with qid */
    marker[qid] = 1; /* this is to prevent self-loops */
    for(k = 0, i = 0; i < nelmnts; i++)
    {
        j = elmntids[i];
        for(ii = eptr[j]; ii < eptr[j + 1]; ii++)
        {
            jj = eind[ii];
            if(marker[jj] == 0)
            {
                nbrs[k++]  = jj;
                marker[jj] = 1;
            }
        }
    }

    /* reset the marker */
    marker[qid] = 0;
    for(i = 0; i < k; i++)
    {
        marker[nbrs[i]] = 0;
    }

    return k;
}


/*************************************************************************/
/*! This function creates and initializes a mesh_t structure */
/*************************************************************************/
mesh_t* CreateMesh(void)
{
    mesh_t* mesh;

    mesh = (mesh_t*)gk_malloc(sizeof(mesh_t), "CreateMesh: mesh");

    InitMesh(mesh);

    return mesh;
}


/*************************************************************************/
/*! This function initializes a mesh_t data structure */
/*************************************************************************/
void InitMesh(mesh_t* mesh)
{
    memset((void*)mesh, 0, sizeof(mesh_t));
}


/*************************************************************************/
/*! This function deallocates any memory stored in a mesh */
/*************************************************************************/
void FreeMesh(mesh_t** r_mesh)
{
    mesh_t* mesh = *r_mesh;

    gk_free((void**)&mesh->eptr, &mesh->eind, &mesh->ewgt, &mesh, LTERM);

    *r_mesh = NULL;
}

/************************ meshpart.c ************************/
/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * meshpart.c
 *
 * This file contains routines for partitioning finite element meshes.
 *
 * Started 9/29/97
 * George
 *
 * $Id: meshpart.c 17513 2014-08-05 16:20:50Z dominique $
 *
 */


/*************************************************************************
* This function partitions a finite element mesh by partitioning its nodal
* graph using KMETIS and then assigning elements in a load balanced fashion.
**************************************************************************/
int METIS_PartMeshNodal(idx_t*  ne,
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
                        idx_t*  npart)
{
    int    sigrval = 0, renumber = 0, ptype;
    idx_t *xadj = NULL, *adjncy = NULL;
    idx_t  ncon = 1, pnumflag = 0;
    int    rstatus = METIS_OK;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;

    renumber = GETOPTION(options, METIS_OPTION_NUMBERING, 0);
    ptype    = GETOPTION(options, METIS_OPTION_PTYPE, METIS_PTYPE_KWAY);

    /* renumber the mesh */
    if(renumber)
    {
        ChangeMesh2CNumbering(*ne, eptr, eind);
        options[METIS_OPTION_NUMBERING] = 0;
    }

    /* get the nodal graph */
    rstatus = METIS_MeshToNodal(ne, nn, eptr, eind, &pnumflag, &xadj, &adjncy);
    if(rstatus != METIS_OK)
        raise(SIGERR);

    /* partition the graph */
    if(ptype == METIS_PTYPE_KWAY)
        rstatus = METIS_PartGraphKway(
            nn, &ncon, xadj, adjncy, vwgt, vsize, NULL, nparts, tpwgts, NULL, options, objval, npart);
    else
        rstatus = METIS_PartGraphRecursive(
            nn, &ncon, xadj, adjncy, vwgt, vsize, NULL, nparts, tpwgts, NULL, options, objval, npart);

    if(rstatus != METIS_OK)
        raise(SIGERR);

    /* partition the other side of the mesh */
    InduceRowPartFromColumnPart(*ne, eptr, eind, epart, npart, *nparts, tpwgts);


SIGTHROW:
    if(renumber)
    {
        ChangeMesh2FNumbering2(*ne, *nn, eptr, eind, epart, npart);
        options[METIS_OPTION_NUMBERING] = 1;
    }

    METIS_Free(xadj);
    METIS_Free(adjncy);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    return metis_rcode(sigrval);
}


/*************************************************************************
* This function partitions a finite element mesh by partitioning its dual
* graph using KMETIS and then assigning nodes in a load balanced fashion.
**************************************************************************/
int METIS_PartMeshDual(idx_t*  ne,
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
                       idx_t*  npart)
{
    int    sigrval = 0, renumber = 0, ptype;
    idx_t  i, j;
    idx_t *xadj = NULL, *adjncy = NULL, *nptr = NULL, *nind = NULL;
    idx_t  ncon = 1, pnumflag = 0;
    int    rstatus = METIS_OK;

    /* set up malloc cleaning code and signal catchers */
    if(!gk_malloc_init())
        return METIS_ERROR_MEMORY;

    gk_sigtrap();

    if((sigrval = gk_sigcatch()) != 0)
        goto SIGTHROW;

    renumber = GETOPTION(options, METIS_OPTION_NUMBERING, 0);
    ptype    = GETOPTION(options, METIS_OPTION_PTYPE, METIS_PTYPE_KWAY);

    /* renumber the mesh */
    if(renumber)
    {
        ChangeMesh2CNumbering(*ne, eptr, eind);
        options[METIS_OPTION_NUMBERING] = 0;
    }

    /* get the dual graph */
    rstatus = METIS_MeshToDual(ne, nn, eptr, eind, ncommon, &pnumflag, &xadj, &adjncy);
    if(rstatus != METIS_OK)
        raise(SIGERR);

    /* partition the graph */
    if(ptype == METIS_PTYPE_KWAY)
        rstatus = METIS_PartGraphKway(
            ne, &ncon, xadj, adjncy, vwgt, vsize, NULL, nparts, tpwgts, NULL, options, objval, epart);
    else
        rstatus = METIS_PartGraphRecursive(
            ne, &ncon, xadj, adjncy, vwgt, vsize, NULL, nparts, tpwgts, NULL, options, objval, epart);

    if(rstatus != METIS_OK)
        raise(SIGERR);


    /* construct the node-element list */
    nptr = ismalloc(*nn + 1, 0, "METIS_PartMeshDual: nptr");
    nind = imalloc(eptr[*ne], "METIS_PartMeshDual: nind");

    for(i = 0; i < *ne; i++)
    {
        for(j = eptr[i]; j < eptr[i + 1]; j++)
            nptr[eind[j]]++;
    }
    MAKECSR(i, *nn, nptr);

    for(i = 0; i < *ne; i++)
    {
        for(j = eptr[i]; j < eptr[i + 1]; j++)
            nind[nptr[eind[j]]++] = i;
    }
    SHIFTCSR(i, *nn, nptr);

    /* partition the other side of the mesh */
    InduceRowPartFromColumnPart(*nn, nptr, nind, npart, epart, *nparts, tpwgts);

    gk_free((void**)&nptr, &nind, LTERM);


SIGTHROW:
    if(renumber)
    {
        ChangeMesh2FNumbering2(*ne, *nn, eptr, eind, epart, npart);
        options[METIS_OPTION_NUMBERING] = 1;
    }

    METIS_Free(xadj);
    METIS_Free(adjncy);

    gk_siguntrap();
    gk_malloc_cleanup(0);

    return metis_rcode(sigrval);
}


/*************************************************************************/
/*! Induces a partitioning of the rows based on a a partitioning of the
    columns. It is used by both the Nodal and Dual routines. */
/*************************************************************************/
void InduceRowPartFromColumnPart(
    idx_t nrows, idx_t* rowptr, idx_t* rowind, idx_t* rpart, idx_t* cpart, idx_t nparts, real_t* tpwgts)
{
    idx_t  i, j, k, me;
    idx_t  nnbrs, *pwgts, *nbrdom, *nbrwgt, *nbrmrk;
    idx_t* itpwgts;

    pwgts  = ismalloc(nparts, 0, "InduceRowPartFromColumnPart: pwgts");
    nbrdom = ismalloc(nparts, 0, "InduceRowPartFromColumnPart: nbrdom");
    nbrwgt = ismalloc(nparts, 0, "InduceRowPartFromColumnPart: nbrwgt");
    nbrmrk = ismalloc(nparts, -1, "InduceRowPartFromColumnPart: nbrmrk");

    iset(nrows, -1, rpart);

    /* setup the integer target partition weights */
    itpwgts = imalloc(nparts, "InduceRowPartFromColumnPart: itpwgts");
    if(tpwgts == NULL)
    {
        iset(nparts, 1 + nrows / nparts, itpwgts);
    }
    else
    {
        for(i = 0; i < nparts; i++)
            itpwgts[i] = 1 + nrows * tpwgts[i];
    }

    /* first assign the rows consisting only of columns that belong to
     a single partition. Assign rows that are empty to -2 (un-assigned) */
    for(i = 0; i < nrows; i++)
    {
        if(rowptr[i + 1] - rowptr[i] == 0)
        {
            rpart[i] = -2;
            continue;
        }

        me = cpart[rowind[rowptr[i]]];
        for(j = rowptr[i] + 1; j < rowptr[i + 1]; j++)
        {
            if(cpart[rowind[j]] != me)
                break;
        }
        if(j == rowptr[i + 1])
        {
            rpart[i] = me;
            pwgts[me]++;
        }
    }

    /* next assign the rows consisting of columns belonging to multiple
     partitions in a  balanced way */
    for(i = 0; i < nrows; i++)
    {
        if(rpart[i] == -1)
        {
            for(nnbrs = 0, j = rowptr[i]; j < rowptr[i + 1]; j++)
            {
                me = cpart[rowind[j]];
                if(nbrmrk[me] == -1)
                {
                    nbrdom[nnbrs] = me;
                    nbrwgt[nnbrs] = 1;
                    nbrmrk[me]    = nnbrs++;
                }
                else
                {
                    nbrwgt[nbrmrk[me]]++;
                }
            }
            ASSERT(nnbrs > 0);

            /* assign it first to the domain with most things in common */
            rpart[i] = nbrdom[iargmax(nnbrs, nbrwgt, 1)];

            /* if overweight, assign it to the light domain */
            if(pwgts[rpart[i]] > itpwgts[rpart[i]])
            {
                for(j = 0; j < nnbrs; j++)
                {
                    if(pwgts[nbrdom[j]] < itpwgts[nbrdom[j]]
                       || pwgts[nbrdom[j]] - itpwgts[nbrdom[j]]
                              < pwgts[rpart[i]] - itpwgts[rpart[i]])
                    {
                        rpart[i] = nbrdom[j];
                        break;
                    }
                }
            }
            pwgts[rpart[i]]++;

            /* reset nbrmrk array */
            for(j = 0; j < nnbrs; j++)
                nbrmrk[nbrdom[j]] = -1;
        }
    }

    gk_free((void**)&pwgts, &nbrdom, &nbrwgt, &nbrmrk, &itpwgts, LTERM);
}
