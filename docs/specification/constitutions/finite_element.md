# Finite Element

**Finite Element** constitutions are the base for vertex-based deformable geometry in libuipc. The state is given by vertex positions; each constitution adds its own energy (or none) on top of the common kinetic and attribute setup. Constitutions such as [Empty](./empty.md), [Particle](./particle.md), [ARAP](./arap.md), [Hookean Spring](./hookean_spring.md), [Neo Hookean Shell](./neo_hookean_shell.md), [Discrete Shell Bending](./discrete_shell_bending.md), [Kirchhoff Rod Bending](./kirchhoff_rod_bending.md), and [Stable Neo Hookean](./stable_neo_hookean.md) inherit this base and document only their additional attributes and energy.

## State Variable

The state variable for a **Finite Element** vertex is its position:

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}.
$$

Positions are the degrees of freedom that evolve over time. Each concrete constitution documents only the attributes it introduces and how they map to its energy formula.
