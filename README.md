# libuipc

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

Both C++ and Python API are provided!

Website: ➡️ https://spirimirror.github.io/libuipc-web/

Documentation: ➡️ https://spirimirror.github.io/libuipc-doc/

Samples: ➡️ https://github.com/spiriMirror/libuipc-samples/


![teaser](docs/media/teaser.png)

## Introduction

**Libuipc** is a library that offers a unified **GPU** incremental potential contact framework for simulating the dynamics of rigid bodies, soft bodies, cloth, and threads, and their couplings. It ensures accurate, **penetration-free frictional contact** and is naturally **differentiable**. Libuipc aims to provide robust and efficient **forward** and **backward** simulations, making it easy for users to integrate with machine learning frameworks, inverse dynamics, robotics, and more.

We are **actively** developing Libuipc and will continue to add more features and improve its performance. We welcome any feedback and contributions from the community!

## Why Libuipc

- **Easy & Powerful**: Libuipc offers an intuitive and unified approach to creating and accessing vivid simulation scenes, supporting a variety of objects and constraints that can be easily added.
- **Fast & Robust**: Libuipc is designed to run fully in parallel on the GPU, achieving high performance and enabling large-scale simulations. It features a robust and accurate frictional contact model that effectively handles complex frictional scenarios without penetration.
- **High Flexibility**: Libuipc provides APIs in both Python and C++ and supports both Linux and Windows systems.
- **Fully Differentiable**: Libuipc provides differentiable simulation APIs for backward optimizations. (Coming Soon)

<table>
  <tr>
    <td>
      <img src="docs/tutorial/media/concepts_code.svg" width="400">
    </td>
    <td>
      <img src="docs/tutorial/media/concepts.drawio.svg" width="450">
    </td>
  </tr>
</table>


## Key Features

- Finite Element-Based Deformable Simulation
- Rigid & Soft Body Strong Coupling Simulation
- Penetration-Free & Accurate Frictional Contact Handling
- User Scriptable Animation Control
- Fully Differentiable Simulation (Diff-Sim Coming Soon)

## News

**2026-2-7**: UIPC now supports PyPI install with `pip install pyuipc`. For the early test version, we support Win/Linux, Python 3.10–3.13 with CUDA 12.8.

**2025-11-01**: The prototype implementation of Libuipc has been open-sourced ([source code](https://github.com/KemengHuang/Stiff-GIPC)) and serves as the performance benchmark for comparisons with our paper.

**2025-5-23**: [StiffGIPC](https://dl.acm.org/doi/10.1145/3735126) will be presented at Siggraph 2025, and Libuipc v1.0.0 will be released soon!

**2024-11-25**: Libuipc v0.9.0 (Alpha) is published! We are excited to share our work with the community. This is a preview version, if you have any feedback or suggestions, please feel free to contact us! [Issues](https://github.com/spiriMirror/libuipc/issues) and [PRs](https://github.com/spiriMirror/libuipc/pulls) are welcome!

## Citation

If you use **Libuipc** in your project, please cite our works:

```
@article{stiffgipc2025,
      author = {Huang, Kemeng and Lu, Xinyu and Lin, Huancheng and Komura, Taku and Li, Minchen},
      title = {StiffGIPC: Advancing GPU IPC for Stiff Affine-Deformable Simulation},
      year = {2025},
      publisher = {Association for Computing Machinery},
      volume = {44},
      number = {3},
      issn = {0730-0301},
      doi = {10.1145/3735126},
      journal = {ACM Trans. Graph.},
      month = may,
      articleno = {31},
      numpages = {20}
}
```

```
@article{gipc2024,
      author = {Huang, Kemeng and Chitalu, Floyd M. and Lin, Huancheng and Komura, Taku},
      title = {GIPC: Fast and Stable Gauss-Newton Optimization of IPC Barrier Energy},
      year = {2024},
      publisher = {Association for Computing Machinery},
      volume = {43},
      number = {2},
      issn = {0730-0301},
      doi = {10.1145/3643028},
      journal = {ACM Trans. Graph.},
      month = {mar},
      articleno = {23},
      numpages = {18}
}
```


