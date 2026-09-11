# Robot hand: pick an apple from the bowl

This example loads the current dining `.blend` and the URDF from sample 87.
It imports the robot through the actual Blender extension operator and bakes
the hand, cloth, ceramics, apples and cutlery together in one libuipc World.
The apple is an unconstrained dynamic ABD body throughout the simulation.

```powershell
& 'D:\Blender\4_5\blender.exe' --background --factory-startup --python-exit-code 1 `
  --python integrations/blender/examples/robot_pick_place.py -- `
  --source output/dining-scene-verified/white_table_setting.blend `
  --output output/robot-hand-apple/rebuild_four_finger `
  --urdf libuipc-samples/assets/sim_data/urdf/robot_hand/robot_hand.urdf `
  --python '<external Python containing CUDA-enabled pyuipc>'
```

`robot_hand_grasp.json` contains the reproducible joint-space grasp. It was
planned from sample 87's actual collision geometry; no additional runtime IK
dependency is needed in Blender or the worker. `plan_hand_grasp.py` can regenerate
candidate poses using NumPy/SciPy and a `robot_model.py` native URDF export. The
planner's forward kinematics is checked against exported native controller poses.

Open `robot_pick_place.blend` with extension 0.3.2 installed and use the timeline.
The saved file selects the final frame and a camera covering the action. The
original dining camera remains available. Relevant frames are:

| Frames | Action |
|---|---|
| 1-50 | Hand holds its initial pose while the dining objects settle |
| 51-110 | Approach above the upper apple |
| 111-150 | Descend with fingers open |
| 151-195 | Close all four fingers, including the thumb |
| 196-250 | Lift approximately 0.22 m |
| 251-325 | Transfer toward the free tabletop region |
| 326-380 | Lower the apple above the cloth |
| 381-420 | Open the fingers and let the apple settle |
| 421-500 | Withdraw the hand |

Edit `Robot root - robot_hand` and `Joint angle ...` transform keyframes to
change the action, then **Bake Simulation**. New target keys invalidate old
cached motion. **Preview Robot Initial Pose** previews the authored start pose;
the bake performs the same alignment. The visible links use the solved MDD
positions and remain independent of the controller hierarchy during playback.
The corrected preset actuates all four chains (the older preset parked joints
8-11 at zero). Root and joint targets now use monotone cubic segments with zero
velocity at each stage boundary. These are changes to the physical drive targets,
not filtering or smoothing of the resulting MDD trajectories.

The 17 links follow sample 87's ABD + `SoftTransformConstraint` design, with
100 MPa rigidity, density 5000 kg/m^3 and translation/rotation strength ratios
10000. They use quasi-static position servos, not a joint-torque dynamics model.
Contact inside the robot assembly is disabled as in sample 87; all robot contacts
against scene objects are enabled with friction 0.8. Existing dining material,
gravity and time-step settings come from the selected `.blend`.
This example explicitly selects the **Converged** solver profile; the library
and plugin defaults are unchanged. Use **Solver Accuracy → Custom** in the
sidebar to edit numerical tolerances and iteration limits. See the Blender user
guide for units, ranges and the exact native-config mapping.

The importer removes two isolated near-zero-volume fragments from the sample
collision meshes and caps 34 open boundary edges on the thumb tip. It preserves
the original asset files. These preparation steps permit closed-mesh validation
without disabling scene sanity checks.

## Checked result

The 500-frame Windows Blender 4.5.3 / Python 3.14 pyuipc 0.9.0 run passed:

- First-50-frame hand displacement below 7.0e-8 m.
- Approximately 0.219 m lift and 0.482 m horizontal transfer.
- All four fingertips move 16-24 mm during closing and maintain contact during
  carrying (sampled surface distances 0.85-6.43 micrometers at frames 250/325).
  The apple has no drive or fixed flag.
- No enabled inter-object crossings, nor crossings between distinct fingertips,
  in any of the 500 cached frames.
- Final apple/cloth surface distance about 1.13 mm; robot separation about 0.242 m.
- Final half-second apple RMS speed about 4.52e-5 m/s; the other four apples remain in the bowl.
- Every physical vertex matches Blender's native cache playback at the checked action frames (maximum error 0).
- Temporal cloth validation passes: carrying-stage hem maximum speed 0.244 m/s
  and maximum acceleration 2.48 m/s²; supported tabletop maximum speed
  0.706 mm/s. Natural free-hem motion remains; the cloth is not frozen or damped.

`pick_place_validation.json` contains the numerical checks, including sampled
surface distances. `pick_place_*.png` shows key stages. Use `--mode inspect` to
reopen and validate an existing bake, or `--mode shots` to render those stages.
Retain the cache directory and derived apple-stem MDD files beside the `.blend`.
`--mode package` builds `robot_pick_place_bundle.zip`. The delivered bundle was
extracted into a different directory and checked in factory Blender without
the addon: all 39 caches matched every vertex at frames 1, 250 and 500, and the
reopened scene rendered correctly. The installed extension's actual UI was also
checked at seven action frames, including its motion-target controls.
The numerical checks concern this assumed-material simulation and its recorded
frames. ABD penalty bodies have finite compliance; these quasi-static position
servos do not establish calibrated real-hand force/torque behavior.

## Why the original cloth kicked during carrying

The old raw cache contained a roughly 8 cm single-frame hem movement at frame
293 (2.45 m/s, maximum acceleration 70.4 m/s²), despite passing crossing and
playback checks. The old third finger being stationary was a separate preset
issue, not a failed URDF import.

The strong loaded robot drives and light cloth share a global nonlinear/PCG
solve. Its native default PCG tolerance is a global preconditioned-residual ratio
(`tol_rate=1e-3`), not a separate accuracy guarantee for every cloth node.
Semi-implicit beta termination additionally permits exiting before ordinary
Newton tolerances pass. Insufficient convergence in this mixed system creates
velocity spikes when the position correction is turned into a time-step velocity.

Replays retained the original three-finger motion, meshes, materials and dt:

| Replay | Maximum cloth speed, frames 251-310 | Maximum acceleration |
|---|---:|---:|
| Original default settings | 2.451 m/s | 70.38 m/s² |
| Only PCG `tol_rate=1e-6` | 0.483 m/s | 17.86 m/s² |
| Converged profile | 0.224 m/s | 2.44 m/s² |

Only disabling semi-implicit mode was insufficient: with the other defaults it
hit the eight-trial line-search limit at output frame 205 (solver substep 816).
Holding the hand stationary removed the carrying-stage kicks. Moving the hand
five meters away was also calm, indicating motion alone is not the trigger;
that last control changes the scene-diagonal-dependent kappa clamp, so it is a
qualitative contact-exclusion check, not an identical-parameter comparison.

The correction uses tighter PCG/Newton convergence and more line-search trials,
plus smoother drive accelerations. All 18 non-robot inputs, including cloth
material/meshes/pins and configured gravity/contact/timestep values, were checked
against the old bake. No cache positions, cloth rest shape, pins or damping were
changed to conceal motion. The four-finger result was subsequently revalidated
through all 500 frames; crossing tests alone are not a temporal stability test.

`replay_robot_bake.py` reproduces controlled tests directly from a bake's exported
inputs in a fresh directory, using the external pyuipc Python. Use `--end-frame
310`, `--profile CONVERGED`, `--linear-tol 1e-6`, `--semi 0`, `--hold`, or
`--robot-offset 5 0 0` for the corresponding experiments. It records distinct
diagnostic fingerprints and owns a child worker so native fatal exits cannot
leave a falsely running status. Diagnostic output is not an authored Blender
cache replacement. These accuracy comparisons are not timing benchmarks.
