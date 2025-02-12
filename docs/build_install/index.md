# Build & Install

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

This page will guide you through building the project.

We use Vcpkg and PyPI to manage the libraries and use CMake to build the project. 

Vcpkg supports both Windows and Linux, we use it to manage the dependencies and keep the consistency of the development environment.

## Prerequisites

The following dependencies are required to build the project.

| Name                                                | Version      | Usage           | Import         |
| --------------------------------------------------- | ------------ | --------------- | -------------- |
| [CMake](https://cmake.org/download/)                | >=3.26       | build system    | system install |
| [Python](https://www.python.org/downloads/)         | >=3.10       | build system    | system install |
| [Cuda](https://developer.nvidia.com/cuda-downloads) | >=12.0       | GPU programming | system install |
| [Vcpkg](https://github.com/microsoft/vcpkg)         | >=2024.04.26 | package manager | git clone      |

## Specific Build Instructions

- Libuipc:
    - [Windows](./windows.md)
    - [Linux](./linux.md)

- Libuipc Documentation:
    - [Build Document](./build_docs.md)