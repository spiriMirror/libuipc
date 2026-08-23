# 04 — Constitution Overview

Headers: `include/uipc/constitution/` (42 files); implementations: `src/constitution/`; mathematical specifications: `docs/specification/constitutions/` (32 documents, including energy formulas and parameter ranges); symbolic derivation: `scripts/symbol_calculation/*.ipynb` (SymEigen generates energy/gradient/Hessian code).

## Basic Mechanisms

- Base classes `IConstitution` / `Constitution` (`constitution.h`); constraint base class `Constraint` (`constraint.h`).
- Unified usage: `constitution.apply_to(geometry, params...)` — writes the UID into `meta().create<U64>(builtin::constitution_uid)` and writes parameters into vertex/instance attributes. The backend claims geometries by UID.
- **UID convention**: official `[0, 2^32-1]`, user-defined `[2^32, 2^64-1]`. Constants are centralized in `include/uipc/builtin/constitution_uid.h`.
- `ElasticModuli` (`elastic_moduli.h`): constructors such as `youngs_poisson(E, nu)` produce Lamé parameters.
- Category base classes: `FiniteElementConstitution` (deformable bodies whose DOFs are vertex positions), `AffineBodyConstitution` (12-DOF affine bodies), `InterAffineBodyConstitution` / `InterPrimitiveConstitution` (inter-body / inter-primitive), `FiniteElementExtraConstitution` / `AffineBodyExtraConstitution` (additional energies).

## The Two Main Base Classes

### Affine Body (ABD, UID 1–8 series, `affine_body.md`)

- State q=(p; a₁;a₂;a₃): translation + rows of the affine matrix, 12 DOF; J is the 3×12 Jacobian.
- Meta attributes: `volume`, `mass_density`, the dyadic mass triple (`abd_mass`, `abd_mass_x_bar`, `abd_mass_x_bar_x_bar`), `inertia`, `dof_offset/count`.
- Instance attributes: `kappa` (recommended 100 MPa–100 GPa), `is_fixed`, `is_dynamic`, `velocity`.
- Variants:
  - #1 OrthoPotential: $V=\kappa\bar v\|AA^T-I\|_F^2$
  - #2 ARAP: $V=\kappa\bar v\|A-R\|_F^2$
  - `AffineBodyShell` / `AffineBodyRod`: codim variants (shell volume = A·2r, rod = πr²L), vertices carry `thickness`.

### Finite Element (FEM, `finite_element.md`)

Vertex positions are the DOFs; Empty/Particle/ARAP/SNH/HookeanSpring etc. all inherit from it.

## Model Catalog (header → description)

### Elastic Bodies
| Header | Description |
|---|---|
| `empty.h` | UID 0, no shape-preservation energy, mass only; more stable and faster when driven entirely by constraints |
| `stable_neo_hookean.h` | Stable Neo-Hookean tetrahedral elasticity, `ElasticModuli::youngs_poisson(E, nu)`. **Since 2026-08-24 this is Stiff-GIPC's SNK1 verbatim** (energy `0.5μ(Ic-3) + 0.5λ(J-1-μ/λ)²`, gradient `μF + (λ(J-1)-μ)·cof(F)`, and the analytic SPD-projected Hessian — twist/flip eigensystem from `math::qr_svd` + a 3×3 direct eigensolve, clamp-and-rebuild; replaces the SymEigen-generated SNH (`0.5λ(J-α)²+0.5μ(Ic-3)-0.5μ·log(Ic+1)`, α=1+0.75μ/λ) + generic 9×9 `make_spd` EVD). Case-88 median 297→266 ms/frame, 89 PCG 213→77/solve |
| `arap.h` | ARAP energy |
| `particle.h` | Mass point (no elasticity) |
| `hookean_spring.h` | Linear spring $E=\frac{\kappa}{2}((L-L_0)/L_0)^2$ |
| `neo_hookean_shell.h` | 2D Neo-Hookean shell |
| `baraff_witkin_shell.h` / `strain_limiting_baraff_witkin.h` | Baraff-Witkin cloth shell and its strain-limited version (`apply_to(sc, stretch_moduli, shear_moduli, ..., strain_rate=100)` dual-moduli overload: stretch/shear use independent (E,ν); the single-modulus overload is retained. Writes triangle attributes `lambda/mu/strain_rate`; **membrane element weight = triangle area (not volume)**, stiffness attributes carry thickness: stretch=`E_s·t/(1-ν_s²)`=(λ_s+2μ_s)·t, shear=μ_sh=`E_sh/(2(1+ν_sh))` which is thickness-independent) |
| `discrete_shell_bending.h` | Discrete shell bending energy (**bending measure = area**; raw `apply_to(sc, κ)` directly gives per-unit-area stiffness; formula overload `apply_to(sc, E, ν)` — **thickness is read from the mesh vertex `thickness` attribute** (averaged over edge endpoints, supports non-uniform shells); a membrane/stretch constitution must be applied first; static helper `bending_stiffness(E,ν,t)`=κ=`E·t³/(12(1-ν²))` literal value) |
| `strain_plastic_discrete_shell_bending.h` / `stress_plastic_discrete_shell_bending.h` | Shell bending with strain/stress plasticity |
| `kirchhoff_rod_bending.h` | Kirchhoff rod bending |

### ABD and ABD Joints (joint axes defined by `linemesh` edges; multi-instance API supports `geo_slots + instance_id + strength_ratio`)
| Header | Description |
|---|---|
| `affine_body_constitution.h` | ABD base class |
| `affine_body_revolute_joint.h` | Revolute joint (1 rotational DOF) |
| `affine_body_revolute_joint_limit.h` | Revolute joint with limits |
| `affine_body_driving_revolute_joint.h` | Driven revolute joint (target angular velocity/angle) |
| `affine_body_revolute_joint_external_force.h` | External force on a revolute joint |
| `affine_body_prismatic_joint.h` (+`_limit`, `_external_force`, `driving_`) | Prismatic joint family |
| `affine_body_spherical_joint.h` | Spherical joint |
| `affine_body_fixed_joint.h` | Fixed joint |
| `affine_body_external_force.h` | ABD external force |

### Constraints (coupled with the Animator, require `is_constrained=1`)
| Header | Description |
|---|---|
| `soft_transform_constraint.h` | Drives an affine body's `aim_transform` |
| `soft_position_constraint.h` | Drives vertices' `aim_position` |
| `external_articulation_constraint.h` | Minimal-coordinate joint system (contributed by Genesis AI, paired with AL-IPC) |

### Soft Stitching (stitch, Inter-primitive)
| Header | Description |
|---|---|
| `soft_vertex_stitch.h` | Vertex-vertex stitch |
| `soft_vertex_edge_stitch.h` | Vertex-edge stitch |
| `soft_vertex_triangle_stitch.h` | Vertex-triangle stitch |

### Others
| Header | Description |
|---|---|
| `finite_element_external_force.h` | FEM external force (additional) |
| `conversion.h` | Utilities such as rigid-body → affine-body conversion |

## Usage Notes

- Parameter range specifications are documented in each `docs/specification/constitutions/*.md` (e.g. $\kappa$: 100 MPa–100 GPa).
- A new material model requires: implementing the `IConstitution` family interface + assigning a UID + writing attributes in `apply_to` + implementing the corresponding SimSystem in the backend to claim the UID + validating/clamping parameters (negative stiffness/density forbidden).
- Joint runtime state such as the angle can be read from edge attributes of the joint geometry (e.g. the `angle` attribute of a revolute joint; see `apps/tests/sim_case/37_abd_revolute_joint.cpp`).
