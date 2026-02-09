# Empty

**Empty** is a special constitution that applies no shape-keeping energy to the geometry. It tells the backend that the geometry is a mass-only element, with no constitutive energy terms.

## #0 Empty

The Empty constitution is particularly useful when users want to drive geometry motion entirely through external constraints (such as [SoftPositionConstraint](./soft_position_constraint.md) or [SoftTransformConstraint](./soft_transform_constraint.md)) without any influence from shape-keeping energies.

By using Empty, the simulation:

- **Remains stable**: No competing energy terms interfere with the constraint-driven motion
- **Runs faster**: No computational cost for evaluating shape-keeping energies
- **Behaves as expected**: The geometry follows the constraints exactly without elastic resistance

This makes Empty ideal for scenarios where you want to input external animations via constraints while maintaining full control over the geometry's motion through the constraint system.
