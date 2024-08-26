#pragma once
/********************************************************************
 * @file   type_define.h
 * @brief  Workaround for the type_define.h in uipc/common/type_define.h
 * 
 * Directly use the type_define.h in uipc/common/type_define.h in cuda backend
 * will cause compilation error. The error is caused by the NVCC Compiler.
 *********************************************************************/
#include <muda/ext/eigen/eigen_cxx20.h>
#include <uipc/common/type_define.h>

#define UIPC_GENERIC MUDA_GENERIC
#define UIPC_DEVICE MUDA_DEVICE
#define UIPC_HOST MUDA_HOST
