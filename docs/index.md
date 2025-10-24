# libuipc




A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

Both **<font color=red>C++</font>** and **<font color=red>Python</font>** APIs are provided!

![](./media/teaser.png)

<div align="center">

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113497581032756&bvid=BV1FMU6Y6Eu2&cid=26807108424&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" height="180" width="180"></iframe>


<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113499007095849&bvid=BV1sBU7YDENs&cid=26812943497&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" height="180" width="180"></iframe>


<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113499007227424&bvid=BV1NBU7YQEGL&cid=26813006592&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" height="180" width="180"></iframe>


<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113542812537310&bvid=BV1iQzcYvEsx&cid=27003194487&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" height="180" width="180"></iframe>

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113787759761628&bvid=BV1pTr9YAEBB&cid=27751613444&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" height="180" width="180"></iframe>

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113800057459136&bvid=BV195c5eXELV&cid=27800175018&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" height="180" width="180"></iframe>
</div>

[More Application](./gallery.md)

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
      <img src="./tutorial/media/concepts_code.svg" width="400">
    </td>
    <td>
      <img src="./tutorial/media/concepts.drawio.svg" width="450">
    </td>
  </tr>
</table>

## Key Features

- Finite Element-Based Deformable Simulation
- Rigid & Soft Body Strong Coupling Simulation
- Penetration-Free & Accurate Frictional Contact Handling
- User Scriptable Animation Control
- Fully Differentiable Simulation (Diff-Sim Coming Soon)

## Document Guidance

- [Build & Install](./build_install/index.md): Instructions to build and install Libuipc on different platforms.
- [Tutorial](./tutorial/index.md): Learn the basic concepts and how to use Libuipc step by step.
- [Specification](./specification/index.md): Detailed definition and explanation of the design and behaviour of Libuipc.

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