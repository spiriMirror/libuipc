# Backend Cuda

This document describes the implementation of the CUDA backend for the `libuipc`. It won't repeat the basic concepts of the `backend`. Before reading this document, it's highly recommended to read the [backend](../backend.md) document first.

In this document, we assume that the reader has a basic understanding of the CUDA programming model, and won't describe the basic concepts of CUDA. You can refer to the [CUDA Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html) for more information. We also require the reader to have a basic understanding of parallel computing and programming, especially the parallel primitives.

## From Local to Global

The cuda backend is almost relying on the [prefix sum](https://en.wikipedia.org/wiki/Prefix_sum) to handle the data aggregation from the local level to the global level.

In the `libuipc`, we have a lot of local data that needs to be aggregated to the global level. 

For example, `GlobalVertexManager` is responsible for managing all the vertices that reported by some `SimSystem`. Every related `SimSystem` reports its local vertices to the `GlobalVertexManager`, and the `GlobalVertexManager` needs to aggregate all the local vertices to the global level and return global offsets to the subsystems.

Say, $N_i$ is the number of the i-th subsystem's local vertices, then the global offset of the i-th subsystem is $S_i=\sum_{j=0}^{i-1} N_j$.

We can apply prefix sum to calculate all the $S_i$ we need.

The aggregation provides a way to allocate the memory only once, which can reduce the memory fragmentation and improve the memory access efficiency, because all the vertices are stored in a continuous memory block.

Almost all the data aggregation in the cuda backend is implemented using the prefix sum. Keep it in mind will help you understand the code better.

The system level prefix sum is calculated on the CPU side, because the system level prefix sum is not very large, it's much faster to calculate it on the CPU side, typically, the `std::exclusive_scan` function in the C++ standard library.

## From Global to Local

In last section, we talked about how to aggregate the local data to the global level. In this section, we will talk about how to distribute the global data to the local level.

### Partition

When some global data needs to be distributed to the local level, we need to partition the global data into several parts, and each part is assigned to a subsystem.

For example, the `GlobalContactManager` collects all the contacts from certain `SimSystem`s, and then distributes the contacts to other `SimSystem`s. Before distributing, we need to partition the contact pairs into several parts, because one `SimSystem` can only process one kind of contact. A vivid exmpale is that the Affine Body-Affine-Body contact can only be processed by the Affine Body subsystem, the Affine Body-FEM contact can only be processed by the ABD-FEM coupling subsystem, the FEM-FEM contact can only be processed by the FEM subsystem. 

This kind of partition is mostly processed on the GPU side, using the parallel partition algorithm.

### Run Length Encoding

If some data is able to be sorted and can be exclusively partitioned, we can also use the run length encoding to distribute the data.

## Separation of Ownership and Access

In the global-local pattern, we always separate the ownership and access of the data. The ownership of the data is always kept by the global manager, the local subsystems only have the access to a subview of the data.

For example, we use `DeviceBuffer<T>` for the global data, and pass the `BufferView<T>` to the local subsystems. The `BufferView<T>` is a subview of the `DeviceBuffer<T>`, containing the offset and the size of the subview.

Such design can improve the memory access efficiency and the safety of the data access. With the safety check from `muda`, an out-of-bound access will be detected and reported.

This separation is applied to all the data in the cuda backend, always keep it in mind when you are reading or writing the code.

## Use Assertion

In GPU programming, it's hard to debug the code, because all the code is running parallelly, and the order of the execution is not guaranteed. 

So always use the assertion to check the correctness of the code when possible. Typically, we use the `MUDA_KERNEL_ASSERT` to check the correctness of the code. E.g.

```cpp
MUDA_KERNEL_ASSERT(x > 0.0, "x should be positive, but x=%f", x);
```

The `MUDA_KERNEL_ASSERT` will check the condition in the kernel, and if the condition is not satisfied, it will print the error message the thread id and the block id, and then terminate the kernel.

## Debug Print

I provide a simple `cout` implementation for the cuda backend, which can be used to print the debug information in the kernel.

```cpp
#include <kernel_cout.h>

// in your kernel or device function
cout << Vector3{1.0, 2.0, 3.0} << "\n";
```

which is a feature provided by the `muda` library, you can refer to the [muda](https://mugdxy.github.io/muda-doc/) for the detailed usage.

The only problem is that, when a kernel is terminated by assertion, the `cout` message will not be kept. So always use the assertion to check the correctness of the code and use the `cout` to debug your calculation.

## Symbol Calculation

I provide an extension of `sympy` called `SymEigen` in `scripts/`, which can be used to calculate the symbolic expression and codegen it to C++ Eigen code. See the `scripts/` for more information. This can release you from the formula typing and reduce the error in the formula.