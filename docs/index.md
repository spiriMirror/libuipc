# libuipc

## Introduction
The libuipc is a C++ library that provides **A Unified Incremental Potential Contact Framework** for simulating the dynamics of rigid bodies, soft bodies, cloth, threads, and so on with penetration-free accurate contact.

The libuipc is designed for the development of physics-based simulation software, such as video games, virtual reality applications, robotics applications, and so on. 

The libuipc is easy to use, high-performance, robust, and highly extensible.

- Easy to use: 
    - The libuipc provides a simple and intuitive API at the frontend, which allows users to easily create and simulate complex physical scenes and seemlessly change the backend implementation without changing the frontend code.
    - The libuipc provides **Python bindings**, which allows users to use the libuipc frontend in python.

- High performance:
    - The libuipc backends are implemented and optimized with Multi-threading, SIMD, and GPU acceleration, which can achieve high performance on modern multi-core CPUs and GPUs.
    - The supported backends include:
        - CUDA GPU backend
        - TBB CPU backend
        - ... [TODO]

- Robust:
    - The libuipc is designed to be robust and stable, which can handle complex physical scenes with fully controllable accuracy.
    - The libuipc system design has high isolation between different modules, which can prevent the error propagation between different subsystems.

- Highly extensible:
    - The libuipc is highly modular, which allows users to easily extend the libuipc with new features.
    - It allows users to add their own custom collision detection algorithms, custom constraints, custom integrators, custom solvers, custom energy functions, and so on.
    - Embedded Python scripting is supported (maybe based on [Numba](https://numba.pydata.org/)), which allows users to easily add custom behaviors to the simulation.

