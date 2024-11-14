# libuipc

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

Both C++ and Python API are provided!

![](./img/teaser.png)

## Introduction

**Libuipc** is a C++ library that provides **A Unified Incremental Potential Contact Framework** for simulating the dynamics of rigid bodies, soft bodies, cloth, threads, and so on with **penetration-free accurate contact**, which is also naturally and intrinsically **differentiable**, allowing users to easily integrate it with machine learning frameworks, inverse dynamics, etc.

## Why Libuipc

- Easy & Powerful: **Libuipc** provides an intuitive and unified way to create/access your vivid simulation scene and supports various types of objects and constraints that you can easily add to your scene.
- Fast & Robust: **Libuipc** is designed to run fully parallel on GPU, which can achieve high performance and large-scale simulation. It also provides a robust and accurate contact model that can handle complex contact scenarios without penetration.
- Highly Extensible: **Libuipc** is designed to be modular and extensible. You can easily extend the backend with your own algorithms.

## Key Features

- Finite Element-Based Deformable Simulation
- Rigid & Soft Body Strong Coupling Simulation
- Penetration-Free & Accurate Contact Handling
- User Scriptable Animation Control
- Fully Differentiable Simulation (Diff-Sim Coming Soon)

## Citation

If you use **Libuipc** in your project, please consider citing our work:

```
@misc{huang2024advancinggpuipcstiff,
      title={Advancing GPU IPC for stiff affine-deformable simulation}, 
      author={Kemeng Huang and Xinyu Lu and Huancheng Lin and Taku Komura and Minchen Li},
      year={2024},
      eprint={2411.06224},
      archivePrefix={arXiv},
      primaryClass={cs.GR},
      url={https://arxiv.org/abs/2411.06224}, 
}
```

