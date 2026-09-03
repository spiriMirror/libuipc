/* Derived from GKlib and adapted for libuipc's C++ build.
 * See ../LICENSE-GKlib.
 *
 * C++ class-template replacement for the GK_MKPQUEUE priority-queue macro.
 *
 * The original macro generated a POD type plus a family of free functions for
 * a binary min-heap indexed by value.  This header turns the same algorithm
 * into a type-safe class template; metis_instantiations.cpp then exposes the
 * original libmetis__ipq_* / libmetis__rpq_* free-function API so that the
 * rest of METIS needs no changes.
 */

#ifndef METIS_PQUEUE_H
#define METIS_PQUEUE_H

#include <cstddef>

#include "GKlib.h"

namespace metis
{
namespace detail
{

    template <typename KeyT, typename ValT, typename KV>
    class PQueue
    {
      public:
        size_t   nnodes   = 0;
        size_t   maxnodes = 0;
        KV*      heap     = nullptr;
        ssize_t* locator  = nullptr;
        KeyT     emptyKey = KeyT{};

        static bool keyHigher(KeyT a, KeyT b) { return a > b; }

        void init(size_t maxnodes_in, const char* heap_msg)
        {
            nnodes   = 0;
            maxnodes = maxnodes_in;
            heap = static_cast<KV*>(gk_malloc(sizeof(KV) * maxnodes, heap_msg));
            locator = reinterpret_cast<ssize_t*>(gk_idxsmalloc(maxnodes, -1, "gk_PQInit: locator"));
        }

        void setEmptyKey(KeyT key) { emptyKey = key; }

        void reset()
        {
            for(ssize_t i = static_cast<ssize_t>(nnodes) - 1; i >= 0; --i)
                locator[heap[i].val] = -1;
            nnodes = 0;
        }

        void free()
        {
            if(heap == nullptr && locator == nullptr)
                return;
            gk_free(reinterpret_cast<void**>(&heap), reinterpret_cast<void**>(&locator), LTERM);
            maxnodes = 0;
        }

        size_t length() const { return nnodes; }

        int insert(ValT node, KeyT key)
        {
            ssize_t i, j;

            ASSERT2(checkHeap());
            ASSERT(locator[node] == -1);

            i = static_cast<ssize_t>(nnodes++);
            while(i > 0)
            {
                j = (i - 1) >> 1;
                if(keyHigher(key, heap[j].key))
                {
                    heap[i]              = heap[j];
                    locator[heap[i].val] = i;
                    i                    = j;
                }
                else
                    break;
            }
            ASSERT(i >= 0);
            heap[i].key   = key;
            heap[i].val   = node;
            locator[node] = i;

            ASSERT2(checkHeap());
            return 0;
        }

        int del(ValT node)
        {
            ssize_t i, j;
            size_t  nnodes_local;
            KeyT    newkey, oldkey;

            ASSERT(locator[node] != -1);
            ASSERT(heap[locator[node]].val == node);
            ASSERT2(checkHeap());

            i             = locator[node];
            locator[node] = -1;

            if(--nnodes > 0 && heap[nnodes].val != node)
            {
                node   = heap[nnodes].val;
                newkey = heap[nnodes].key;
                oldkey = heap[i].key;

                if(keyHigher(newkey, oldkey))
                {
                    /* filter up */
                    while(i > 0)
                    {
                        j = (i - 1) >> 1;
                        if(keyHigher(newkey, heap[j].key))
                        {
                            heap[i]              = heap[j];
                            locator[heap[i].val] = i;
                            i                    = j;
                        }
                        else
                            break;
                    }
                }
                else
                {
                    /* filter down */
                    nnodes_local = nnodes;
                    while((j = (i << 1) + 1) < static_cast<ssize_t>(nnodes_local))
                    {
                        if(keyHigher(heap[j].key, newkey))
                        {
                            if(j + 1 < static_cast<ssize_t>(nnodes_local)
                               && keyHigher(heap[j + 1].key, heap[j].key))
                                j++;
                            heap[i]              = heap[j];
                            locator[heap[i].val] = i;
                            i                    = j;
                        }
                        else if(j + 1 < static_cast<ssize_t>(nnodes_local)
                                && keyHigher(heap[j + 1].key, newkey))
                        {
                            j++;
                            heap[i]              = heap[j];
                            locator[heap[i].val] = i;
                            i                    = j;
                        }
                        else
                            break;
                    }
                }

                heap[i].key   = newkey;
                heap[i].val   = node;
                locator[node] = i;
            }

            ASSERT2(checkHeap());
            return 0;
        }

        void update(ValT node, KeyT newkey)
        {
            ssize_t i, j;
            size_t  nnodes_local;
            KeyT    oldkey;

            oldkey = heap[locator[node]].key;
            if(!keyHigher(newkey, oldkey) && !keyHigher(oldkey, newkey))
                return;

            ASSERT(locator[node] != -1);
            ASSERT(heap[locator[node]].val == node);
            ASSERT2(checkHeap());

            i = locator[node];

            if(keyHigher(newkey, oldkey))
            {
                /* filter up */
                while(i > 0)
                {
                    j = (i - 1) >> 1;
                    if(keyHigher(newkey, heap[j].key))
                    {
                        heap[i]              = heap[j];
                        locator[heap[i].val] = i;
                        i                    = j;
                    }
                    else
                        break;
                }
            }
            else
            {
                /* filter down */
                nnodes_local = nnodes;
                while((j = (i << 1) + 1) < static_cast<ssize_t>(nnodes_local))
                {
                    if(keyHigher(heap[j].key, newkey))
                    {
                        if(j + 1 < static_cast<ssize_t>(nnodes_local)
                           && keyHigher(heap[j + 1].key, heap[j].key))
                            j++;
                        heap[i]              = heap[j];
                        locator[heap[i].val] = i;
                        i                    = j;
                    }
                    else if(j + 1 < static_cast<ssize_t>(nnodes_local)
                            && keyHigher(heap[j + 1].key, newkey))
                    {
                        j++;
                        heap[i]              = heap[j];
                        locator[heap[i].val] = i;
                        i                    = j;
                    }
                    else
                        break;
                }
            }

            heap[i].key   = newkey;
            heap[i].val   = node;
            locator[node] = i;

            ASSERT2(checkHeap());
        }

        ValT getTop()
        {
            ssize_t i, j;
            ValT    vtx, node;
            KeyT    key;

            ASSERT2(checkHeap());

            if(nnodes == 0)
                return static_cast<ValT>(-1);

            nnodes--;

            vtx          = heap[0].val;
            locator[vtx] = -1;

            if((i = static_cast<ssize_t>(nnodes)) > 0)
            {
                key  = heap[i].key;
                node = heap[i].val;
                i    = 0;
                while((j = 2 * i + 1) < static_cast<ssize_t>(nnodes))
                {
                    if(keyHigher(heap[j].key, key))
                    {
                        if(j + 1 < static_cast<ssize_t>(nnodes)
                           && keyHigher(heap[j + 1].key, heap[j].key))
                            j = j + 1;
                        heap[i]              = heap[j];
                        locator[heap[i].val] = i;
                        i                    = j;
                    }
                    else if(j + 1 < static_cast<ssize_t>(nnodes)
                            && keyHigher(heap[j + 1].key, key))
                    {
                        j                    = j + 1;
                        heap[i]              = heap[j];
                        locator[heap[i].val] = i;
                        i                    = j;
                    }
                    else
                        break;
                }

                heap[i].key   = key;
                heap[i].val   = node;
                locator[node] = i;
            }

            ASSERT2(checkHeap());
            return vtx;
        }

        ValT seeTopVal() const
        {
            return (nnodes == 0 ? static_cast<ValT>(-1) : heap[0].val);
        }

        KeyT seeTopKey() const
        {
            return (nnodes == 0 ? emptyKey : heap[0].key);
        }

        KeyT seeKey(ValT node) const { return heap[locator[node]].key; }

        int checkHeap() const
        {
            ssize_t i, j;
            size_t  nnodes_local;
            ssize_t count = 0;

            nnodes_local = nnodes;

            if(nnodes_local == 0)
                return 1;

            ASSERT(locator[heap[0].val] == 0);
            for(i = 1; i < static_cast<ssize_t>(nnodes_local); i++)
            {
                ASSERT(locator[heap[i].val] == i);
                ASSERT(!keyHigher(heap[i].key, heap[(i - 1) / 2].key));
            }
            for(i = 1; i < static_cast<ssize_t>(nnodes_local); i++)
                ASSERT(!keyHigher(heap[i].key, heap[0].key));

            for(j = i = 0; i < static_cast<ssize_t>(maxnodes); i++)
            {
                if(locator[i] != -1)
                    count++;
            }
            ASSERTP(count == static_cast<ssize_t>(nnodes_local),
                    ("%" PRIDX " %" PRIDX "\n", count, static_cast<ssize_t>(nnodes_local)));

            return 1;
        }
    };

}  // namespace detail
}  // namespace metis

#endif  // METIS_PQUEUE_H
