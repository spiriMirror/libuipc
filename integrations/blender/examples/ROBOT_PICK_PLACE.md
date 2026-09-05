# Robot hand: pick an apple from the bowl

This example loads the current dining `.blend` and the URDF from sample 87.
It imports the robot through the actual Blender extension operator and bakes
the hand, cloth, ceramics, apples and cutlery together in one libuipc World.
The apple is an unconstrained dynamic ABD body throughout the simulation.

```powershell
& 'D:\Blender\4_5\blender.exe' --background --factory-startup --python-exit-code 1 `
  --python integrations/blender/examples/robot_pick_place.py -- `
  --source output/dining-scene-verified/white_table_setting.blend `
  --output output/robot-hand-apple/run_01 `
  --urdf libuipc-samples/assets/sim_data/urdf/robot_hand/robot_hand.urdf `
  --python '<external Python containing CUDA-enabled pyuipc>'
```

`robot_hand_grasp.json` contains the reproducible joint-space grasp. It was
planned from sample 87's actual collision geometry; no additional runtime IK
dependency is needed in Blender or the worker. `plan_hand_grasp.py` can regenerate
candidate poses using NumPy/SciPy and a `robot_model.py` native URDF export. The
planner's forward kinematics is checked against exported native controller poses.

Open `robot_pick_place.blend` with extension 0.3 installed and use the timeline.
The saved file selects the final frame and a camera covering the action. The
original dining camera remains available. Relevant frames are:

| Frames | Action |
|---|---|
| 1-50 | Hand holds its initial pose while the dining objects settle |
| 51-110 | Approach above the upper apple |
| 111-150 | Descend with fingers open |
| 151-195 | Close thumb and two fingers |
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

The 17 links follow sample 87's ABD + `SoftTransformConstraint` design, with
100 MPa rigidity, density 5000 kg/m^3 and translation/rotation strength ratios
10000. They use quasi-static position servos, not a joint-torque dynamics model.
Contact inside the robot assembly is disabled as in sample 87; all robot contacts
against scene objects are enabled with friction 0.8. Existing dining material,
gravity and time-step settings come from the selected `.blend`.

The importer removes two isolated near-zero-volume fragments from the sample
collision meshes and caps 34 open boundary edges on the thumb tip. It preserves
the original asset files. These preparation steps permit closed-mesh validation
without disabling scene sanity checks.

## Checked result

The 500-frame Windows Blender 4.5.3 / Python 3.14 pyuipc 0.9.0 run passed:

- First-50-frame hand displacement below 7.1e-8 m.
- Approximately 0.22 m lift and 0.48 m horizontal transfer.
- Three fingertip contacts while carrying the apple; no apple drive or fixed flag.
- No enabled inter-object triangle-surface crossings in any cached frame.
- Final apple/cloth surface distance about 1.13 mm; robot separation about 0.260 m.
- Final half-second apple RMS speed about 3.2e-5 m/s; the other four apples remain in the bowl.
- Every physical vertex matches Blender's native cache playback at the checked action frames (maximum error 0).

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
frames; they do not establish calibrated real-hand actuator behavior.
