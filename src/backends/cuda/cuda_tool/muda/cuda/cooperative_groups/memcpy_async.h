#pragma once

#ifdef __INTELLISENSE__
#include <muda/cuda/details/hack_cuda_define>
#undef _COOPERATIVE_GROUPS_MEMCPY_ASYNC
#endif
#include <cooperative_groups/memcpy_async.h>
