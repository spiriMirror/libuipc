# External Articulation Constraint

## 23 ExternalArticulationConstraint


External Constraint is designed to incorporate external kinetic-like energy into the IPC system. Typically,`ExternalArticulationConstraint` is used to control the motion of a set of affine bodies by prescribing articulation DOFs (degrees of freedom) such as joint positions, joint angles, etc.

For a **single** Articulation(say a robot arm), the kinetic term is defined as:

$$
K = \frac{1}{2}\left( \delta\boldsymbol{\theta}(\mathbf{q}, \mathbf{q}^t) - \tilde{\delta\boldsymbol{\theta}} \right)^T \mathbf{M}^t \left(\delta\boldsymbol{\theta}(\mathbf{q}, \mathbf{q}^t) -\tilde{\delta\boldsymbol{\theta}} \right)
$$

where $\mathbf{q}_{12n}$ is the DOF of $n$ affine bodies, $\delta\boldsymbol{\theta}_{m}$ is the variational DOF of $m$ Joints, $\tilde{\delta\boldsymbol{\theta}}$ is the predicted variational joint DOF , $\mathbf{M}^t$ is the **Effective Mass** on the articulation DOF, which is a $m \times m$ dense matrix. The $\mathbf{M}^t$ with $t$ denotes that it is the effective mass at previous time step, indicating that it is a constant during the current time step's optimization.

It's **users' responsibility** to pass the $\mathbf{M}^t$ and $\tilde{\delta\boldsymbol{\theta}}$ to the `ExternalArticulationConstraint` through the `libuipc` animator callback function. The details of computing $\mathbf{M}^t$ and $\tilde{\delta\boldsymbol{\theta}}$ are out of scope of this document, as they depend on the specific articulation system being modeled. Some recommended references for computing these terms include:

- [A Quick Tutorial on Multibody Dynamics](https://github.com/dartsim/dart/blob/main/docs/dynamics.pdf)

Say, users are using a physics engine like [Mujoco](https://mujoco.org/), [DART](https://dart.readthedocs.io/en/latest/) or so on, to simulate the articulation system, they can extract the effective mass matrix and predicted variational joint DOFs from the physics engine at each time step, and pass them to `libuipc` for collision handling. Thus, we can achieve a stable coupling between the IPC collision handling and the articulation system simulation.

### Joint Constraint

With the kinetic term only is not sufficient to fully constrain the affine bodies to follow the articulation motion. We need to add additional DOF constraints to ensure that the affine bodies move according to the articulation DOFs.

Supported DOF constraints include:

- [Affine Body Revolute Joint](./affine_body_revolute_joint.md)
- [Affine Body Prismatic Joint](./affine_body_prismatic_joint.md)

![External Articulation Constraint](./media/external_articulation_constraint_fig1.drawio.svg)

In revolute joint case,  $\hat{\mathbf{n}}$ is the normal vector perpendicular to the joint axis, $\hat{\mathbf{b}}$ is the binormal vector of the joint axis, and $\hat{\mathbf{t}}$ is the direction vector along the joint axis pointed into the figure.

The angle between two bodies around the joint axis is defined as:

$$
\cos\theta = \frac{
    \hat{\mathbf{b}}_i \cdot \hat{\mathbf{b}}_j + \hat{\mathbf{n}}_i \cdot \hat{\mathbf{n}}_j
    }{2},
$$

$$
\sin\theta = \frac{
    \hat{\mathbf{n}}_i \cdot \hat{\mathbf{b}}_j - \hat{\mathbf{b}}_i \cdot \hat{\mathbf{n}}_j
    }{2}.
$$

to be symmetric, we consider both $\hat{\mathbf{b}}$ and $\hat{\mathbf{n}}$ in the definition. And we can derive the angle $\theta$ as:

$$
\delta\theta = \arctan \left(\tan \left(\theta - \theta^t \right)\right) = \arctan \frac{\sin\theta \cos\theta^t - \cos\theta \sin\theta^t}{\cos\theta \cos\theta^t + \sin\theta \sin\theta^t},
$$

where $\theta^t$ is the joint angle at previous time step, the $\sin\theta^t$ and $\cos\theta^t$ can be calculated using the previous frame vectors $\hat{\mathbf{b}}^t$ and $\hat{\mathbf{n}}^t$s.

In prismatic joint case, the translation along the joint axis is defined as:

$$
\theta = \frac{
    (\mathbf{c}_j - \mathbf{c}_i) \cdot \hat{\mathbf{t}}_i -
    (\mathbf{c}_i - \mathbf{c}_j) \cdot \hat{\mathbf{t}}_j
}{2},
$$

to be symmetric, we consider both directions $\hat{\mathbf{t}}_i$ and $\hat{\mathbf{t}}_j$ in the definition.

The variational joint DOF is then defined as:
$$
\delta\theta = \theta - \theta^t,
$$

where $\theta^t$ is the joint translation at previous time step, can be calculated using the frame vectors $\mathbf{c}^t$ and $\hat{\mathbf{t}}^t$s.

## Reference Previous DOF

To prevent a uncontrollable drift of motion without motion synchronization between the external system and the IPC system, we support the input of `ref_dof_prev` attribute on the affine body instance. The `ref_dof_prev` attribute is a vector of 12 elements ([Affine Body DOF](./affine_body.md)), representing the previous DOF of the affine body. If this attribute is provided, the `ExternalArticulationConstraint` will use it to synchronize the motion between the external system and the IPC system based on the previous state from the external system.

Thus the formula for the variational joint DOF is:

$$
K = \frac{1}{2}\left( \delta\boldsymbol{\theta}(\mathbf{q}, \mathbf{q}^t_{ref}) - \tilde{\delta\boldsymbol{\theta}} \right)^T \mathbf{M}^t \left(\delta\boldsymbol{\theta}(\mathbf{q}, \mathbf{q}^t_{ref}) -\tilde{\delta\boldsymbol{\theta}} \right)
$$

where $\mathbf{q}^t_{ref}$ is the previous DOF of the affine body calculated from the external system. Users are responsible for providing and update the `ref_dof_prev` attribute on the affine body instance.

