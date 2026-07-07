# Affine Body Revolute Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #18 AffineBodyRevoluteJoint

The **Affine Body Revolute Joint** constitution constrains two affine bodies to rotate relative to each other about a shared axis. This joint allows for rotational motion while restricting translational movement between the two bodies.

![Affine Body Revolute Joint](./media/affine_body_revolute_joint_fig1.drawio.svg)

We assume 2 affine body indices $i$ and $j$, each with their own state vector (to be concerete, transform) $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

The joint axis in world space is defined by 2 points $\mathbf{x}^0$ and $\mathbf{x}^1$. At the beginning of the simulation, the relationships are kept:

$$
\mathbf{x}^0 = \mathbf{J}^0_i \mathbf{q}_i = \mathbf{J}^0_j \mathbf{q}_j,
$$

and,

$$
\mathbf{x}^1 = \mathbf{J}^1_i \mathbf{q}_i = \mathbf{J}^1_j \mathbf{q}_j,
$$

intuitively $\mathbf{J}^0$ and $\mathbf{J}^1$ tell the local coordinates of the two points on affine bodies $i$ and $j$. 

The energy function for the **Affine Body Revolute Joint** is defined as:

$$
E = \frac{K}{2} \cdot \left( 
    \| \mathbf{J}^0_i \mathbf{q}_i - \mathbf{J}^0_j \mathbf{q}_j \|^2
    +
    \| \mathbf{J}^1_i \mathbf{q}_i - \mathbf{J}^1_j \mathbf{q}_j \|^2
\right),
$$

where $K$ is the stiffness constant of the joint, we choose $K=\gamma (m_i + m_j)$, where $\gamma$ is a **user defined** `strength_ratio` parameter, and $m_i$ and $m_j$ are the masses of the two affine bodies.

## Angle State

The revolute joint tracks a scalar joint angle $\theta$ shared by the extra-per-joint constitutions (driving, limit, external torque). For each body $k \in \{i,j\}$ the backend builds an orthonormal basis $(\hat{\mathbf{t}}_k, \hat{\mathbf{n}}_k, \hat{\mathbf{b}}_k)$ from the hinge geometry (see `sym/affine_body_revolute_joint`): $\hat{\mathbf{t}}_k$ is the **unit tangent** along body $k$’s hinge segment and $\hat{\mathbf{b}}_k = \hat{\mathbf{t}}_k \times \hat{\mathbf{n}}_k$, so $[\hat{\mathbf{n}}_k, \hat{\mathbf{b}}_k]$ is the rotating perpendicular frame used downstream. Current relative angle is

$$
\cos\theta = \frac{\hat{\mathbf{n}}_i \cdot \hat{\mathbf{n}}_j + \hat{\mathbf{b}}_i \cdot \hat{\mathbf{b}}_j}{2}, \quad
\sin\theta = \frac{\hat{\mathbf{b}}_i \cdot \hat{\mathbf{n}}_j - \hat{\mathbf{n}}_i \cdot \hat{\mathbf{b}}_j}{2},
$$

$$
\theta = \operatorname{atan2}(\sin\theta,\; \cos\theta) \in (-\pi, \pi].
$$

The sign follows the right-hand rule **about the child tangent** $+\hat{\mathbf{t}}_{\mathrm{child}}$ (see below): counterclockwise when viewed along $+\hat{\mathbf{t}}_{\mathrm{child}}$.

### Parent vs. child along the axis

Map **left geometry** (`l_*`, body $i$) to **parent** and **right geometry** (`r_*`, body $j$) to **child**, matching typical kinematic-tree layout.

At a valid configuration, the mapped joint axis rays are **anti-parallel** on the shared line:

$$
\hat{\mathbf{t}}_{\mathrm{parent}} = \hat{\mathbf{t}}_i = -\,\hat{\mathbf{t}}_{\mathrm{child}} = -\,\hat{\mathbf{t}}_j,
$$

because each body carries its edge from attachment point $\mathbf{x}^0_k$ to $\mathbf{x}^1_k$ while the pairing links endpoints across bodies along the same physical axis.

Angular state $\theta$, `angle`, limits, and driving use **both** bodies’ bases symmetrically via the formulas above. Which ray defines **positive external torque** is specified **only** for constitution **Affine Body Revolute Joint External Force** (UID 668): actuator torque follows **child-positive** $+\hat{\mathbf{t}}_{\mathrm{child}}$; see [Parent vs. child axis and positive torque](./affine_body_revolute_joint_external_force.md#parent-vs-child-axis-and-positive-torque).

### State Update

At the end of each time step, the backend advances a continuously accumulated (unwrapped) reference angle and writes it back to the `angle` edge attribute:

$$
r^{(n)} = r^{(n-1)} + \operatorname{rem}\!\left(\theta(\mathbf{q}^{(n)}) - r^{(n-1)}\right), \qquad
\theta_{\text{current}}^{(n)} = r^{(n)} + \alpha_0,
$$

where $\theta(\mathbf{q}^{(n)})$ is the wrapped angle from the formulas above at step $n$, $\operatorname{rem}(x)$ is the representative of $x$ modulo $2\pi$ closest to $0$, and $\alpha_0$ is `init_angle`, a **user-facing offset** shifting the reported "zero" (default `0.0`). Seeded at $r^{(0)} = 0$, so $\theta_{\text{current}}^{(0)} = \alpha_0$.

Thus $\theta_{\text{current}}$ is a **continuous, multi-turn absolute angle** that accumulates across turns instead of wrapping at $\pm\pi$. The unwrap is exact while the per-step rotation stays below $\pi$ — this is an aliasing limit of sampling the wrapped angle at step endpoints, not an implementation choice: from two wrapped samples a $+\Delta$ and a $\Delta-2\pi$ rotation are indistinguishable. Fast *continuous* rotation that would exceed $\pi$ per step is handled by shrinking the time step so each sub-step stays below $\pi$. A genuine *discontinuous* jump (e.g. a reset/teleport) carries no winding information to recover; for it, write the authoritative angle (with winding) into `angle` **before** pushing the new body state through the accessor — the backend adopts the modified entries on re-sync and republishes; untouched joints are unaffected.

The `angle` attribute is thus available to any frontend and to all extra-per-joint constitutions (driving, limit, external torque). On an external DOF write (e.g. `AffineBodyStateAccessorFeature`), the backend re-syncs `current_angles` synchronously via `GlobalJointDofManager`.

#### Convention for all user-facing bounds

All user-facing angle quantities on this joint (`limit/lower`, `limit/upper` on [AffineBodyRevoluteJointLimit](./affine_body_revolute_joint_limit.md); `aim_angle` on [AffineBodyDrivingRevoluteJoint](./affine_body_driving_revolute_joint.md)) are expressed in the same "`current_*` frame" as the reported `angle` attribute. A user can therefore set them directly against the value they read back from `angle`, without having to reason about `init_angle`. Wherever `init_angle` appears inside a solver-side formula, it is **subtracted** from the user-facing quantity to translate it into the raw frame $\theta(\mathbf{q})$.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint):

- `l_geo_id` (`IndexT`): scene geometry slot id for the left body $i$
- `r_geo_id` (`IndexT`): scene geometry slot id for the right body $j$
- `l_inst_id` (`IndexT`): instance index within the left geometry
- `r_inst_id` (`IndexT`): instance index within the right geometry
- `strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$ above
- `angle` (`Float`): $\theta_{\text{current}}$, current relative joint angle in radians. A continuous, unbounded, multi-turn absolute angle (not wrapped to $(-\pi, \pi]$) — see [State Update](#state-update). Written by the backend every time step (read-only for the user).
- `init_angle` (`Float`): $\alpha_0$, initial angle offset added to the extracted relative angle. Default `0.0`.

When the joint is created via Local `create_geometry`, optional **edge** attributes (each `Vector3`) supply local joint geometry: `l_position0`, `l_position1` (left body local frame), `r_position0`, `r_position1` (right body local frame).
