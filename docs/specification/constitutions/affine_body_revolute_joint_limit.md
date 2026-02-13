# Affine Body Revolute Joint Limit

## #670 AffineBodyRevoluteJointLimit

`AffineBodyRevoluteJointLimit` is an **InterAffineBody extra constitution** for
`AffineBodyRevoluteJoint` (`UID=18`).

It adds a unilateral cubic penalty on the revolute joint angle:

$$
E(x)=
\begin{cases}
0, & l \le x \le u \\
s(x-u)^3, & x>u \\
s(l-x)^3, & x<l
\end{cases}
$$

where:
- $l$ is lower angle limit,
- $u$ is upper angle limit,
- $s$ is `strength`.

To avoid directly optimizing inverse trigonometric angle recovery, the angle is represented by:

$$
x = \theta_0 + \delta
$$

$$
\theta_0 = \Delta \Theta(\mathbf{q}^{t-1}, \mathbf{q}_{ref}), \quad
\delta = \Delta \Theta(\mathbf{q}, \mathbf{q}^{t-1})
$$

where $\mathbf{q}_{ref}$ is reference pose captured at initialization.

For revolute joint, $x$ is in radians.

## Apply Order

You must apply base revolute joint first, then apply limit:

1. `AffineBodyRevoluteJoint.apply_to(...)`
2. `AffineBodyRevoluteJointLimit.apply_to(...)`

If base joint is missing (or not `UID=18`), `apply_to` asserts.

## Stored Attributes

On joint edges:
- `limit/lower`
- `limit/upper`
- `limit/strength`

And limit UID (`670`) is inserted into:
- `meta.extra_constitution_uids`

## API

C++:

```cpp
AffineBodyRevoluteJointLimit limit;
limit.apply_to(joint_mesh, lower, upper, strength);
```

Python:

```python
limit = AffineBodyRevoluteJointLimit()
limit.apply_to(joint_mesh, lower, upper, strength)
```

## Notes

- Inside `[lower, upper]`, energy/gradient/Hessian are zero.
- At boundaries (`x==lower` or `x==upper`), value and derivatives are zero.
- `strength` is used directly as energy coefficient.
