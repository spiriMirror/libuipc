"""
Revolute joint limit GUI demo.

This demo builds two parallel revolute joint pairs:
- Left pair: base revolute joint only (no limit).
- Right pair: base revolute joint + AffineBodyRevoluteJointLimit.

Both pairs are driven by the same reciprocal (sinusoidal) angular motor command.
The GUI panel shows:
- commanded angular velocity,
- measured relative angle for both sides,
- limit range and out-of-range violation on the limited side.

Run:
    python python/examples/revolute_joint_limit_gui_demo.py
"""

import os
import sys
import numpy as np
import polyscope as ps
import polyscope.imgui as psim

from uipc import Logger, Matrix4x4, Engine, World, Scene, SceneIO, Animation, view, builtin
from uipc.geometry import (
    SimplicialComplex,
    SimplicialComplexIO,
    label_surface,
    label_triangle_orient,
    flip_inward_triangles,
    linemesh,
)
from uipc.constitution import (
    AffineBodyConstitution,
    AffineBodyRevoluteJoint,
    AffineBodyRevoluteJointLimit,
    RotatingMotor,
)

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "tests"))
from asset import AssetDir

REVOLUTE_DRIVE_FREQ_HZ = 0.35
REVOLUTE_DRIVE_VEL_AMP = 3.2


def process_surface(sc: SimplicialComplex) -> SimplicialComplex:
    """Prepare mesh surface labels/orientation for stable visualization and simulation."""
    label_surface(sc)
    label_triangle_orient(sc)
    return flip_inward_triangles(sc)


def signed_angle_about_z(reference: np.ndarray, current: np.ndarray) -> float:
    """Compute signed angle from reference to current vector around +Z."""
    a = reference[:2]
    b = current[:2]

    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1.0e-10 or nb < 1.0e-10:
        return 0.0

    a = a / na
    b = b / nb

    cross_z = a[0] * b[1] - a[1] * b[0]
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    return float(np.arctan2(cross_z, dot))


def build_scene():
    """Create world/scene and return objects needed by GUI update callbacks."""
    Logger.set_level(Logger.Level.Warn)

    workspace = AssetDir.output_path(__file__)
    engine = Engine("cuda", workspace)
    world = World(engine)

    config = Scene.default_config()
    config["dt"] = 0.01
    config["gravity"] = [[0.0], [0.0], [0.0]]
    config["contact"]["enable"] = False
    scene = Scene(config)

    abd = AffineBodyConstitution()

    pre = Matrix4x4.Identity()
    pre[0, 0] = 0.22
    pre[1, 1] = 0.22
    pre[2, 2] = 0.22

    io = SimplicialComplexIO(pre)
    link_mesh = io.read(f"{AssetDir.tetmesh_path()}/cube.msh")
    link_mesh = process_surface(link_mesh)
    link_mesh.instances().resize(4)

    abd.apply_to(link_mesh, 1.0e8)

    transforms = view(link_mesh.transforms())

    t0 = Matrix4x4.Identity()
    t0[0:3, 3] = np.array([-0.40, 0.0, 0.0])
    transforms[0] = t0

    t1 = Matrix4x4.Identity()
    t1[0:3, 3] = np.array([-1.00, 0.0, 0.0])
    transforms[1] = t1

    t2 = Matrix4x4.Identity()
    t2[0:3, 3] = np.array([1.00, 0.0, 0.0])
    transforms[2] = t2

    t3 = Matrix4x4.Identity()
    t3[0:3, 3] = np.array([0.40, 0.0, 0.0])
    transforms[3] = t3

    is_fixed = view(link_mesh.instances().find(builtin.is_fixed))
    is_fixed[:] = 0
    is_fixed[1] = 1
    is_fixed[3] = 1

    links = scene.objects().create("links")
    link_slot = links.geometries().create(link_mesh)[0]

    motor = RotatingMotor()
    motor.apply_to(link_slot.geometry(),
                   strength=30.0,
                   motor_axis=np.array([0.0, 0.0, 1.0], dtype=np.float32),
                   motor_rot_vel=0.0)

    revolute = AffineBodyRevoluteJoint()
    revolute_limit = AffineBodyRevoluteJointLimit()

    no_limit_joint = linemesh(
        np.array([[-0.70, 0.0, -0.25], [-0.70, 0.0, 0.25]], dtype=np.float32),
        np.array([[0, 1]], dtype=np.int32),
    )
    revolute.apply_to(no_limit_joint,
                      [link_slot], [0],
                      [link_slot], [1],
                      [100.0])

    with_limit_joint = linemesh(
        np.array([[0.70, 0.0, -0.25], [0.70, 0.0, 0.25]], dtype=np.float32),
        np.array([[0, 1]], dtype=np.int32),
    )
    revolute.apply_to(with_limit_joint,
                      [link_slot], [2],
                      [link_slot], [3],
                      [100.0])

    lower, upper = -0.35, 0.35
    revolute_limit.apply_to(with_limit_joint, lower, upper, 350.0)

    joints = scene.objects().create("joints")
    joints.geometries().create(no_limit_joint)
    joints.geometries().create(with_limit_joint)

    def drive_rotating_motor(info: Animation.UpdateInfo):
        """Drive both revolute pairs with the same reciprocal angular command."""
        geo = info.geo_slots()[0].geometry()
        time = info.frame() * info.dt()
        drive_vel = REVOLUTE_DRIVE_VEL_AMP * np.sin(2.0 * np.pi * REVOLUTE_DRIVE_FREQ_HZ * time)

        is_constrained = view(geo.instances().find(builtin.is_constrained))
        is_constrained[:] = 0
        is_constrained[0] = 1
        is_constrained[2] = 1

        motor_vel = view(geo.instances().find("motor_rot_vel"))
        motor_vel[:] = 0.0
        motor_vel[0] = drive_vel
        motor_vel[2] = drive_vel

        RotatingMotor.animate(geo, info.dt())

    scene.animator().insert(links, drive_rotating_motor)

    world.init(scene)

    return engine, world, scene, link_slot, lower, upper


def run_gui_demo():
    """Launch Polyscope UI and step simulation interactively."""
    engine, world, scene, link_slot, lower, upper = build_scene()
    sio = SceneIO(scene)

    transforms0 = view(link_slot.geometry().transforms())
    marker = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    ref_no_limit = np.array(transforms0[0][0:3, 0:3]) @ marker
    ref_with_limit = np.array(transforms0[2][0:3, 0:3]) @ marker

    # Keep a strong reference to engine alive during GUI loop.
    state = {"run": False, "engine": engine}

    ps.init()
    ps.set_ground_plane_mode("none")

    surf = sio.simplicial_surface()
    v = surf.positions().view()
    t = surf.triangles().topo().view()
    mesh = ps.register_surface_mesh("abd_revolute_joint_limit", v.reshape(-1, 3), t.reshape(-1, 3))
    mesh.set_edge_width(1.0)

    def sample_values():
        """Read current relative angles and violation metric."""
        tf = view(link_slot.geometry().transforms())

        cur_no_limit = np.array(tf[0][0:3, 0:3]) @ marker
        cur_with_limit = np.array(tf[2][0:3, 0:3]) @ marker

        no_limit_theta = signed_angle_about_z(ref_no_limit, cur_no_limit)
        with_limit_theta = signed_angle_about_z(ref_with_limit, cur_with_limit)

        violation = max(0.0, with_limit_theta - upper) + max(0.0, lower - with_limit_theta)
        return no_limit_theta, with_limit_theta, violation

    def update_mesh():
        s = sio.simplicial_surface()
        vv = s.positions().view()
        mesh.update_vertex_positions(vv.reshape(-1, 3))

    def on_update():
        if psim.Button("run / pause"):
            state["run"] = not state["run"]

        psim.SameLine()
        if psim.Button("step"):
            world.advance()
            world.retrieve()
            update_mesh()

        if state["run"]:
            world.advance()
            world.retrieve()
            update_mesh()

        no_limit_theta, with_limit_theta, violation = sample_values()
        time = world.frame() * 0.01
        drive_vel = REVOLUTE_DRIVE_VEL_AMP * np.sin(2.0 * np.pi * REVOLUTE_DRIVE_FREQ_HZ * time)

        psim.Separator()
        psim.Text("Revolute limit demo (reciprocal drive)")
        psim.Text("Left: no limit, Right: with limit")
        psim.Text(f"Frame: {world.frame()}")
        psim.Text(f"Commanded angular velocity: {drive_vel:+.3f} rad/s")
        psim.Text(f"No limit angle: {np.degrees(no_limit_theta):+.2f} deg")
        psim.Text(f"With limit angle: {np.degrees(with_limit_theta):+.2f} deg")
        psim.Text(
            f"Limit range: [{np.degrees(lower):+.1f}, {np.degrees(upper):+.1f}] deg"
        )
        psim.Text(f"With-limit violation: {violation:.6f} rad")

    ps.set_user_callback(on_update)
    ps.show()


if __name__ == "__main__":
    run_gui_demo()
