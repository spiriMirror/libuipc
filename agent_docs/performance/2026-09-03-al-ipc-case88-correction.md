# AL-IPC sample 88 trajectory correction (2026-09-03)

## Scope

Sample 88 is a mixed-scale stress case: two volumetric FEM bunnies, a
4,225-vertex cloth, frictional self/contact response, a half-plane, and the MAS
preconditioner. It exposed a large AL-IPC trajectory mismatch that small rigid
or single-contact tests did not show.

Measurements below used an RTX 5090, CUDA 13.2, Release CMake build, Python
3.12, `dt=0.01`, `K_min=6`, MAS, and the sample's original material/contact
parameters. The local sample worktree had cloth creation commented during
diagnosis, so validation restored that one call in memory; it did not overwrite
the concurrent sample edit. Raw diagnostic logs remain under ignored
`output/` and are not repository artifacts.

## Root causes

Three independent defects amplified one another:

1. `advance_al.cu` ignored global semi-implicit `K_min`. A full CCD fraction of
   one therefore ended many frames after one outer solve. Standard IPC still
   took enough Newton work to follow the intended stiff transient.
2. EE friction computed edge-edge relative displacement and energy but mapped
   its derivative with `point_triangle_jacobi`. The resulting vector was finite
   but was not the derivative of the reported energy.
3. The newly selected `diag_norm` default broadcast one uniform penalty to
   cloth and volumetric vertices with very different masses and stiffnesses.
   On this scene it generated repeated non-descent directions. This is a
   limitation of the current uniform scaling implementation, not evidence that
   Hessian-informed AL scaling is intrinsically invalid.

A fourth safety defect let the final untested backtracking trial become the
next state after a line-search limit. One observed run then grew from finite
energy to approximately `4.1e15` and failed in the following CCD query.

## Corrections

- AL keeps its remaining initial-state weight at one for the first
  `K_min - 1` completed outer updates and starts `(1 - alpha)` attenuation at
  update `K_min`. `beta_tol` remains standard-IPC-only; AL uses
  `contact/al-ipc/toi_threshold`.
- EE friction uses `edge_edge_jacobi`. At the time of this measurement, AL's
  negative threshold made its parallel-EE predicate false. The finalized owner
  policy applies the same disabled threshold to AL normal contact. Standard
  IPC retains positive `1e-3` detection, using mollified normal derivatives and
  skipping detected parallel friction pairs.
- `per_vertex` mass-based scaling is again the default. `diag_norm` remains an
  explicit experimental comparison mode.
- Line search checks its final permitted trial, rejects non-finite energy, and
  restores the recorded start point before warning or throwing.

## Matched trajectory evidence

The original one-step AL path was deceptively fast but diverged before the main
contact event. Against a 60-frame IPC run with identical scene inputs:

| Run | Mean ms/frame | Newton total | Frame-60 upper-centroid error | Frame-60 lower-centroid error |
| --- | ---: | ---: | ---: | ---: |
| IPC reference | 149.56 | 325 | 0 | 0 |
| old AL behavior | 45.85 | 129 | 0.2963 | 0.0807 |
| AL with corrected `K_min` | 154.10 | 482 | 0.0115 | 0.0045 |

At the three-decimal trajectory-log resolution, corrected AL and IPC match
through frame 24. Upper/lower centroid errors at frame 60 are approximately
0.011/0.004, versus 0.297/0.081 before the fix. Exact late-frame equality is
not an acceptance criterion: IPC and AL solve different contact objectives,
and the pile-up becomes chaotic after sustained contact.

The penalty-mode isolation after the friction fixes was decisive:

| AL mode, 100 frames | Mean ms/frame | Line-search-limit frames | Non-converged frames |
| --- | ---: | ---: | ---: |
| uniform `diag_norm` | 965.20 | 44 | 44 |
| mass-based `per_vertex` | 152.28 | 0 | 0 |

The final corrected default, with AL's parallel-edge mollifier disabled,
completed all 250 frames in 153.96 ms/frame mean. Every frame converged with no
line-search limit, Newton limit, or runtime error. Final upper/lower centroids
were `(-0.6552, -0.8712, 0.0871)` and
`(-0.0299, -0.8898, 0.1598)`, respectively. Both bodies remained on the
floor-scale contact region instead of the old prematurely terminated fall.

## Regression boundary

Keep the following checks together when changing AL friction, penalty scaling,
or solve boundaries:

- finite-difference PT, EE, and half-plane friction gradients;
- positive-threshold standard IPC detection and negative-threshold AL rejection;
- AL cumulative-progress unit tests at `K_min=1` and `K_min=6`;
- a simulation assertion that an AL contact frame honors configured `K_min`;
- sample 88 with cloth enabled, inspecting structured `frame_stats()` for line
  search/Newton limits as well as the trajectory.

Do not use frame time alone as the AL acceptance signal. The broken one-step
path was faster precisely because it skipped required outer work.
