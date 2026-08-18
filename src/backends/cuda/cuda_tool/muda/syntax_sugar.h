#pragma once

// usage:
// Launch().apply(
// [] $()
// {
//
// });
//
// you don't need to write mutable and __device__
#define $(...) MUDA_DEVICE(__VA_ARGS__) mutable

// usage:
// Launch().apply(
// [$def(viewer, buffer)] $()
// {
//
// });
#define $def(viewer, from) viewer = (from).name(#viewer)

// usage:
// ComputeGraph g;
// g.$node(name)
// {

// };
//
// you don't need to write g.create_node(name) << [&]
// {
//
// }
#define $node(name) create_node(name) << [&]

// usage:
// ParallelFor()
// .$kernel_name()
#define $kernel_name() kernel_name(__FUNCTION__)

// usage:
// ParallelFor()
// .$file_line()
#define $file_line() file_line(__FILE__, __LINE__)
