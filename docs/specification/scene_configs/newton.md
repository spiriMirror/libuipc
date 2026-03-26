# Newton Solver Details

## Convergence Criteria

Each Newton step, the solver checks two conditions:

1. **Displacement tolerance** — maximum vertex displacement must satisfy:

$$\|\Delta x\|_\infty < \texttt{velocity\_tol} \times \texttt{dt}$$

2. **CCD tolerance** — the continuous collision detection step size $\alpha_{\text{ccd}}$ must satisfy:

$$\alpha_{\text{ccd}} \geq \texttt{ccd\_tol}$$

When `ccd_tol = 1.0` (default), the solver requires that a full Newton step is collision-free before converging. Lowering this value relaxes the requirement — useful when exact CCD convergence is unnecessary.

## Affine Body Rotation Tolerance

For affine bodies, convergence additionally requires:

$$\|\Delta q\|_\infty < \texttt{transrate\_tol} \times \texttt{dt}$$

where $\Delta q$ is the change in affine body degrees of freedom. The default `transrate_tol = 0.1 /s` with `dt = 0.01` gives an absolute tolerance of `0.001`.

## Iteration Bounds

- `max_iter` (default 1024): hard cap on Newton iterations. If reached, a warning is issued (or an exception if `extras/strict_mode/enable = 1`).
- `min_iter` (default 1): minimum iterations before early exit is allowed. Relevant for semi-implicit mode.

## Semi-Implicit Mode

When `newton/semi_implicit/enable = 1`, the solver uses a $\beta$-schedule that blends implicit and explicit updates:

1. Run at least `min_iter` Newton iterations.
2. After `min_iter`, compute $\beta$ from the residual ratio.
3. Terminate when $\beta \leq \texttt{beta\_tol}$.

This trades strict convergence for speed — useful for real-time or interactive scenarios where frame budget matters more than per-step accuracy.
