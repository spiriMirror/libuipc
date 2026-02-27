#pragma once

#ifdef __INTELLISENSE__
// This is a hack to make intellisense work with cooperative_groups.h
#include <muda/cuda/details/hack_cuda_define>
#undef _COOPERATIVE_GROUPS_H_
#endif
#include <cooperative_groups.h>