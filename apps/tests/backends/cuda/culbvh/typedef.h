//
// Created by birdpeople on Fri Aug 04 2023, 16:14:42
// Copyright (c) 2023
//

#pragma once
#ifndef TYPE_DEF_H
#define TYPE_DEF_H
#include <string>

#if __has_include("cuda_runtime.h")
#pragma nv_diag_suppress esa_on_defaulted_function_ignored
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>


#define checkCudaErrors(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true) {
	if (code != cudaSuccess) {
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

#endif

    #ifdef __CUDACC__
        #define CUDA_CALLABLE __host__ __device__
        #define DEVICE_CALLABLE __device__
        #define HOST_CALLABLE __host__
        #define DEVICE_INLINE_CALLABLE __device__ __forceinline__
        #define HOST_INLINE_CALLABLE __host__ __forceinline__
        #define CUDA_INLINE_CALLABLE __host__ __device__ __forceinline__
        #define INLINE_CALLABLE __forceinline__
    #else
        #define CUDA_CALLABLE
        #define DEVICE_CALLABLE
        #define HOST_CALLABLE
        #define DEVICE_INLINE_CALLABLE
        #define CUDA_INLINE_CALLABLE inline
        #define HOST_INLINE_CALLABLE inline
        #define INLINE_CALLABLE inline
    #endif


namespace culbvh {
#ifdef CULBVH_ASSET_PATH
    static const std::string get_asset_path() {
        return std::string{CULBVH_ASSET_PATH};
    }
#else
    static const std::string get_asset_path() {
        return std::string{};
    }
#endif

using namespace std;

	inline void print(int x, const char* format = "%d") {
		printf(format, x);
	}

	CUDA_INLINE_CALLABLE void printDiv(const char* label = nullptr) {
		if (!label) label = "div";
		printf("\n----%s----\n", label);
	}

	CUDA_INLINE_CALLABLE void print(float x, const char* format = "%f\n") {
		printf(format, x);
	}

	CUDA_INLINE_CALLABLE void print(double x, const char* format = "%f\n") {
		printf(format, x);
	}

	CUDA_INLINE_CALLABLE void print(const float3& v, const char* format = "%.4f") {
		printf("{");
		printf(format, v.x);
		printf(", ");
		printf(format, v.y);
		printf(", ");
		printf(format, v.z);
		printf("}\n");
	}

	CUDA_INLINE_CALLABLE void print(const double3& v, const char* format = "%.4f") {
		printf("{");
		printf(format, v.x);
		printf(", ");
		printf(format, v.y);
		printf(", ");
		printf(format, v.z);
		printf("}\n");
	}


}

#endif // TYPE_DEF_H
