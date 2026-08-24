# Constitution UID

The Constitution UID is a unique identifier for a constitution known by `libuipc`, which is a 64-bit unsigned integer. 

The official constitution UID has a range of $[0, 2^{32}-1]$. The range $[2^{32}, 2^{64}-1]$ is reserved for user-defined constitutions.

Every official constitution will be documented in this specification. A user-defined constitution can apply for an official constitution UID by submitting a pull request to the `libuipc` repository, After code review, the constitution will be added to the official constitution list.

The related documentation of the constitution will be added to the [Constutitions/](./constitutions/index.md) directory.

When applying a constitution to a geometry, the `constitution_uid` attribute of the `meta` attribute of the geometry will be set to the constitution UID. The backend will use this UID to determine the constitution of the geometry.

## Official UID List

<!-- AUTO-GENERATED UID TABLE: BEGIN (scripts/gen_uid_doc.py) -->

| UID | Name | Type | Source |
| --- | ---- | ---- | ------ |
| 0 | Empty | FiniteElement | `src/constitution/empty.cpp` |
| 1 | OrthoPotential | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 2 | ARAP | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 3 | AffineBody | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 4 | AffineBody | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 5 | AffineBody | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 6 | AffineBody | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 7 | AffineBody | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 8 | AffineBody | AffineBody | `src/constitution/affine_body_constitution.cpp` |
| 9 | ARAP | FiniteElement | `src/constitution/arap.cpp` |
| 10 | StableNeoHookean | FiniteElement | `src/constitution/stable_neo_hookean.cpp` |
| 11 | NeoHookeanShell | FiniteElement | `src/constitution/neo_hookean_shell.cpp` |
| 12 | HookeanSpring | FiniteElement | `src/constitution/hookean_spring.cpp` |
| 13 | Particle | FiniteElement | `src/constitution/particle.cpp` |
| 14 | SoftPositionConstraint | Constraint | `src/constitution/soft_position_constraint.cpp` |
| 15 | KirchhoffRodBending | FiniteElement | `src/constitution/kirchhoff_rod_bending.cpp` |
| 16 | SoftTransformConstraint | Constraint | `src/constitution/soft_transform_constraint.cpp` |
| 17 | DiscreteShellBending | FiniteElement | `src/constitution/discrete_shell_bending.cpp` |
| 18 | AffineBodyRevoluteJoint | InterAffineBody | `src/constitution/affine_body_revolute_joint.cpp` |
| 19 | AffineBodyDrivingRevoluteJoint | Constraint | `src/constitution/affine_body_driving_revolute_joint.cpp` |
| 20 | AffineBodyPrismaticJoint | InterAffineBody | `src/constitution/affine_body_prismatic_joint.cpp` |
| 21 | AffineBodyDrivingPrismaticJoint | Constraint | `src/constitution/affine_body_driving_prismatic_joint.cpp` |
| 22 | SoftVertexStitch | InterPrimitive | `src/constitution/soft_vertex_stitch.cpp` |
| 23 | ExternalArticulationConstitution | InterAffineBody | `src/constitution/external_articulation_constraint.cpp` |
| 24 | ExternalArticulationConstraint | Constraint | `src/constitution/external_articulation_constraint.cpp` |
| 25 | AffineBodyFixedJoint | InterAffineBody | `src/constitution/affine_body_fixed_joint.cpp` |
| 26 | AffineBodySphericalJoint | InterAffineBody | `src/constitution/affine_body_spherical_joint.cpp` |
| 27 | AffineBodyDrivingSphericalJoint | Constraint | `src/constitution/affine_body_driving_spherical_joint.cpp` |
| 28 | AffineBodyD6Joint | InterAffineBody | `src/constitution/affine_body_d6_joint.cpp` |
| 29 | SoftVertexEdgeStitch | InterPrimitive | `src/constitution/soft_vertex_edge_stitch.cpp` |
| 30 | SoftVertexTriangleStitch | InterPrimitive | `src/constitution/soft_vertex_triangle_stitch.cpp` |
| 31 | StrainPlasticDiscreteShellBending | FiniteElement | `src/constitution/strain_plastic_discrete_shell_bending.cpp` |
| 32 | StressPlasticDiscreteShellBending | FiniteElement | `src/constitution/stress_plastic_discrete_shell_bending.cpp` |
| 666 | AffineBodyExternalForce | Constraint | `src/constitution/affine_body_external_force.cpp` |
| 667 | AffineBodyPrismaticJointExternalForce | Constraint | `src/constitution/affine_body_prismatic_joint_external_force.cpp` |
| 668 | AffineBodyRevoluteJointExternalForce | Constraint | `src/constitution/affine_body_revolute_joint_external_force.cpp` |
| 669 | AffineBodyPrismaticJointLimit | InterAffineBody | `src/constitution/affine_body_prismatic_joint_limit.cpp` |
| 670 | AffineBodyRevoluteJointLimit | InterAffineBody | `src/constitution/affine_body_revolute_joint_limit.cpp` |
| 671 | FiniteElementExternalForce | Constraint | `src/constitution/finite_element_external_force.cpp` |
| 819 | StrainLimitingBaraffWitkinShell | FiniteElement | `src/constitution/strain_limiting_baraff_witkin.cpp` |

<!-- AUTO-GENERATED UID TABLE: END -->
