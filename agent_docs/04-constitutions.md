# 04 — 本构模型（constitution）全览

头文件：`include/uipc/constitution/`（42 个）；实现：`src/constitution/`；数学规范：`docs/specification/constitutions/`（32 篇，含能量公式与参数范围）；符号推导：`scripts/symbol_calculation/*.ipynb`（SymEigen 生成能量/梯度/Hessian 代码）。

## 基础机制

- 基类 `IConstitution` / `Constitution`（`constitution.h`）；约束基类 `Constraint`（`constraint.h`）。
- 使用方式统一为 `constitution.apply_to(geometry, params...)`：把 UID 写入 `meta().create<U64>(builtin::constitution_uid)`，参数写入顶点/instance 属性。后端按 UID 认领几何。
- **UID 约定**：官方 `[0, 2^32-1]`，用户自定义 `[2^32, 2^64-1]`。常量集中在 `include/uipc/builtin/constitution_uid.h`。
- `ElasticModuli`（`elastic_moduli.h`）：`youngs_poisson(E, nu)` 等构造 Lamé 参数。
- 分类基类：`FiniteElementConstitution`（顶点位置为自由度的可变形体）、`AffineBodyConstitution`（12-DOF 仿射体）、`InterAffineBodyConstitution` / `InterPrimitiveConstitution`（体间/图元间）、`FiniteElementExtraConstitution` / `AffineBodyExtraConstitution`（附加能）。

## 两大主基类

### Affine Body（ABD，UID 1–8 系列，`affine_body.md`）

- 状态 q=(p; a₁;a₂;a₃)：平移 + 仿射矩阵行，12 DOF；J 为 3×12 雅可比。
- Meta 属性：`volume`、`mass_density`、dyadic mass 三元组（`abd_mass`、`abd_mass_x_bar`、`abd_mass_x_bar_x_bar`）、`inertia`、`dof_offset/count`。
- Instance 属性：`kappa`（建议 100 MPa–100 GPa）、`is_fixed`、`is_dynamic`、`velocity`。
- 变体：
  - #1 OrthoPotential：$V=\kappa\bar v\|AA^T-I\|_F^2$
  - #2 ARAP：$V=\kappa\bar v\|A-R\|_F^2$
  - `AffineBodyShell` / `AffineBodyRod`：codim 变体（壳体积 = A·2r，杆 = πr²L），顶点带 `thickness`。

### Finite Element（FEM，`finite_element.md`）

顶点位置为自由度；Empty/Particle/ARAP/SNH/HookeanSpring 等均继承。

## 模型清单（头文件 → 说明）

### 弹性体
| 头文件 | 说明 |
|---|---|
| `empty.h` | UID 0，无形状保持能仅质量；完全由约束驱动时更稳更快 |
| `stable_neo_hookean.h` | Stable Neo-Hookean 四面体弹性，`ElasticModuli::youngs_poisson(E, nu)` |
| `arap.h` | ARAP 能量 |
| `particle.h` | 质点（无弹性） |
| `hookean_spring.h` | 线性弹簧 $E=\frac{\kappa}{2}((L-L_0)/L_0)^2$ |
| `neo_hookean_shell.h` | 2D Neo-Hookean 壳 |
| `baraff_witkin_shell.h` / `strain_limiting_baraff_witkin.h` | Baraff-Witkin 布料壳及其应变限制版（`apply_to(sc, stretch_moduli, shear_moduli, ..., strain_rate=100)` 双模量重载：stretch/shear 用独立 (E,ν)；单模量重载保留。写入三角形属性 `lambda/mu/strain_rate`；**膜元权重=三角形 area（非体积）**，刚度属性自带厚度：stretch=`E_s·t/(1-ν_s²)`=(λ_s+2μ_s)·t、shear=μ_sh=`E_sh/(2(1+ν_sh))` 与厚度无关） |
| `discrete_shell_bending.h` | 离散壳弯曲能（**弯曲测度=面积**；raw `apply_to(sc, κ)` 直接给单位面积刚度；公式重载 `apply_to(sc, E, ν)`——**厚度读网格 vertex `thickness` 属性**（边端点平均，支持非均匀壳），需先 apply 膜/stretch 本构；静态帮助 `bending_stiffness(E,ν,t)`=κ=`E·t³/(12(1-ν²))` 字面值） |
| `strain_plastic_discrete_shell_bending.h` / `stress_plastic_discrete_shell_bending.h` | 应变/应力塑性的壳弯曲 |
| `kirchhoff_rod_bending.h` | Kirchhoff 杆弯曲 |

### ABD 与 ABD 关节（`linemesh` 边定义关节轴，多实例 API 支持 `geo_slots + instance_id + strength_ratio`）
| 头文件 | 说明 |
|---|---|
| `affine_body_constitution.h` | ABD 基类 |
| `affine_body_revolute_joint.h` | 铰链（1 旋转自由度） |
| `affine_body_revolute_joint_limit.h` | 带限位铰链 |
| `affine_body_driving_revolute_joint.h` | 驱动铰链（目标角速度/角度） |
| `affine_body_revolute_joint_external_force.h` | 铰链外力 |
| `affine_body_prismatic_joint.h` (+`_limit`, `_external_force`, `driving_`) | 移动副系列 |
| `affine_body_spherical_joint.h` | 球铰 |
| `affine_body_fixed_joint.h` | 固定连接 |
| `affine_body_external_force.h` | ABD 外力 |

### 约束（与 Animator 耦合，需 `is_constrained=1`）
| 头文件 | 说明 |
|---|---|
| `soft_transform_constraint.h` | 驱动 affine body 的 `aim_transform` |
| `soft_position_constraint.h` | 驱动顶点 `aim_position` |
| `external_articulation_constraint.h` | 最小坐标关节系统（Genesis AI 贡献，AL-IPC 配套） |

### 软缝合（stitch，Inter-primitive）
| 头文件 | 说明 |
|---|---|
| `soft_vertex_stitch.h` | 顶点-顶点缝合 |
| `soft_vertex_edge_stitch.h` | 顶点-边缝合 |
| `soft_vertex_triangle_stitch.h` | 顶点-三角形缝合 |

### 其他
| 头文件 | 说明 |
|---|---|
| `finite_element_external_force.h` | FEM 外力（附加） |
| `conversion.h` | 刚体→仿射体转换等工具 |

## 使用要点

- 参数范围规范写在各 `docs/specification/constitutions/*.md`（如 $\kappa$: 100 MPa–100 GPa）。
- 新材料模型需：实现 `IConstitution` 族接口 + 分配 UID + `apply_to` 写属性 + 在后端实现对应 SimSystem 认领 UID + 验证/钳制参数（禁负刚度/密度）。
- 关节角度等运行状态可从关节几何的边属性读取（如 revolute joint 的 `angle` 属性，见 `apps/tests/sim_case/37_abd_revolute_joint.cpp`）。
