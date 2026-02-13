# Affine Body Prismatic Joint Limit

## #669 AffineBodyPrismaticJointLimit

`AffineBodyPrismaticJointLimit` is an **InterAffineBody extra constitution** for
`AffineBodyPrismaticJoint` (`UID=20`).

It adds a unilateral cubic penalty on the prismatic joint value:

$$
E(x)=
\begin{cases}
0, & l \le x \le u \\
s(x-u)^3, & x>u \\
s(l-x)^3, & x<l
\end{cases}
$$

where:
- $l$ is lower limit,
- $u$ is upper limit,
- $s$ is `strength`.

During optimization, the joint value uses the same delta formulation as external articulation:

$$
x = \theta_0 + \delta
$$

$$
\theta_0 = \Delta \Theta(\mathbf{q}^{t-1}, \mathbf{q}_{ref}), \quad
\delta = \Delta \Theta(\mathbf{q}, \mathbf{q}^{t-1})
$$

For prismatic joint, $x$ has length unit.

## Apply Order

You must apply base prismatic joint first, then apply limit:

1. `AffineBodyPrismaticJoint.apply_to(...)`
2. `AffineBodyPrismaticJointLimit.apply_to(...)`

If base joint is missing (or not `UID=20`), `apply_to` asserts.

## Stored Attributes

On joint edges:
- `limit/lower`
- `limit/upper`
- `limit/strength`

And limit UID (`669`) is inserted into:
- `meta.extra_constitution_uids`

## API

C++:

```cpp
AffineBodyPrismaticJointLimit limit;
limit.apply_to(joint_mesh, lower, upper, strength);
```

Python:

```python
limit = AffineBodyPrismaticJointLimit()
limit.apply_to(joint_mesh, lower, upper, strength)
```

## Notes

- Inside `[lower, upper]`, energy/gradient/Hessian are zero.
- At boundaries (`x==lower` or `x==upper`), value and derivatives are zero.
- `strength` is used directly as energy coefficient.
