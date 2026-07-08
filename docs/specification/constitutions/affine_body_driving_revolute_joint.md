# Affine Body Driving Revolute Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #19 AffineBodyDrivingRevoluteJoint

The **Affine Body Driving Revolute Joint** is a constraint constitution that drives a [Revolute Joint](./affine_body_revolute_joint.md) (UID=18) to a target angle. It must be applied to a geometry that already has an `AffineBodyRevoluteJoint` constitution.

The driving joint supports two operating modes:

- **Active mode** (`is_passive = 0`): the joint drives toward a user-specified `aim_angle`.
- **Passive mode** (`is_passive = 1`): the joint resists external forces by treating the current angle as the target, effectively locking the joint in place.

The constraint can also be toggled on and off at runtime via the `driving/is_constrained` flag.

## Energy

We assume 2 affine body indices $i$ and $j$, each with their own state vector $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

The current relative rotation angle $\theta$ between the two bodies about the joint axis is extracted from the affine body states using the per-body $(\hat{\mathbf{n}}_k, \hat{\mathbf{b}}_k)$ basis maintained by the base [Revolute Joint](./affine_body_revolute_joint.md):

$$
\cos\theta = \frac{\hat{\mathbf{n}}_i \cdot \hat{\mathbf{n}}_j + \hat{\mathbf{b}}_i \cdot \hat{\mathbf{b}}_j}{2}, \quad
\sin\theta = \frac{\hat{\mathbf{b}}_i \cdot \hat{\mathbf{n}}_j - \hat{\mathbf{n}}_i \cdot \hat{\mathbf{b}}_j}{2},
$$

$$
\theta = \operatorname{atan2}(\sin\theta,\; \cos\theta) \in (-\pi, \pi],
$$

where $\hat{\mathbf{n}}_k$, $\hat{\mathbf{b}}_k$ are the normal and binormal directions in body $k$'s current frame, obtained from the stored rest-space basis via the affine map. The sign follows the right-hand rule around $+\hat{\mathbf{t}}$.

For the energy below, this measurement is used through its continuously-unwrapped counterpart, anchored to the base joint's `current_angles` committed at the step start (see [State Update](./affine_body_revolute_joint.md#state-update)), so it does not jump by $2\pi$ when a Newton trial crosses the $\operatorname{atan2}$ branch cut. It agrees with $\theta$ modulo $2\pi$, exactly so while the rotation since the last commit is below $\pi$.

`aim_angle` is a **user-facing target** expressed in the same frame as the reported `angle` (i.e. $\theta_{\text{current}}$). To translate it into the raw $\theta$-space actually used by the energy, the solver subtracts $\theta_{\text{init}}$ (the base joint's `init_angle`):

$$
\tilde\theta =
\begin{cases}
\theta_{\text{aim}} - \theta_{\text{init}}, & \text{is\_passive} = 0 \\
\theta_{\text{current}} - \theta_{\text{init}}, & \text{is\_passive} = 1
\end{cases}
$$

In active mode, $\tilde\theta$ is the raw-angle counterpart of `aim_angle`. In passive mode, `aim_angle` is silently replaced by the current reported angle, which gives $\tilde\theta \approx \theta(\mathbf{q})$ and therefore a near-zero energy — the joint locks at the instant state and resists further motion.

The energy function is a quadratic penalty on the raw-angle difference:

$$
E = \frac{K}{2} \left(\theta - \tilde\theta\right)^2,
$$

where $K = \gamma (m_i + m_j)$, $\gamma$ is the **user defined** `driving/strength_ratio` parameter, and $m_i$, $m_j$ are the masses of the two affine bodies. Substituting the definitions, the penalty is equivalent to $\tfrac{K}{2}(\theta_{\text{current}} - \theta_{\text{aim}})^2$ in the user-facing frame — the driving joint simply pulls the reported `angle` toward `aim_angle`.

Because $\theta$ is continuously unwrapped, $\theta - \tilde\theta$ stays smooth across the $\pm\pi$ branch cut, so `aim_angle` is an **unbounded absolute target**: values beyond $\pm\pi$ are reached over multiple steps, not advanced incrementally by the caller. The precondition (per-step rotation below $\pi$; resets need an explicit `angle` write) is that of the base joint's [State Update](./affine_body_revolute_joint.md#state-update).

When `driving/is_constrained = 0`, the energy is zero and the driving effect is disabled.

The current angle $\theta_{\text{current}}$ and the initial offset $\theta_{\text{init}}$ are read from the **base** [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md), which tracks them on the `angle` / `init_angle` edge attributes and keeps them in sync with the body state. The driving joint only consumes these values — it does not own them.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md) (UID=18) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). The edge inherits all linking and state fields of the base [Affine Body Revolute Joint](./affine_body_revolute_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, `strength_ratio`, `angle`, `init_angle`, and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

Driving-specific attributes on **edges**:

- `driving/strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$ above
- `driving/is_constrained`: enables (`1`) or disables (`0`) the driving effect
- `is_passive`: passive mode (`1`) locks at the current angle; active mode (`0`) drives to `aim_angle`
- `aim_angle`: $\theta_{\text{aim}}$, the target angle in active mode
