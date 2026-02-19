"""
Prismatic joint limit GUI demo.

This demo builds two parallel prismatic joint pairs:
- Left pair: base prismatic joint only (no limit).
- Right pair: base prismatic joint + AffineBodyPrismaticJointLimit.

Both pairs are driven by the same reciprocal (sinusoidal) linear motor command.
The GUI panel shows:
- commanded motor velocity,
- measured joint displacement for both sides,
- limit range and out-of-range violation on the limited side.

Run:
    python python/examples/prismatic_joint_limit_gui_demo.py
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
    AffineBodyPrismaticJoint,
    AffineBodyPrismaticJointLimit,
    LinearMotor,
)

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "tests"))
from asset import AssetDir

PRISMATIC_DRIVE_FREQ_HZ = 0.35
PRISMATIC_DRIVE_VEL_AMP = 1.0


def process_surface(sc: SimplicialComplex) -> SimplicialComplex:
    """Prepare mesh surface labels/orientation for stable visualization and simulation."""
    label_surface(sc)
    label_triangle_orient(sc)
    return flip_inward_triangles(sc)


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
    t0[0:3, 3] = np.array([-1.00, 0.0, 0.0])
    transforms[0] = t0

    t1 = Matrix4x4.Identity()
    t1[0:3, 3] = np.array([-0.20, 0.0, 0.0])
    transforms[1] = t1

    t2 = Matrix4x4.Identity()
    t2[0:3, 3] = np.array([0.20, 0.0, 0.0])
    transforms[2] = t2

    t3 = Matrix4x4.Identity()
    t3[0:3, 3] = np.array([1.00, 0.0, 0.0])
    transforms[3] = t3

    is_fixed = view(link_mesh.instances().find(builtin.is_fixed))
    is_fixed[:] = 0
    is_fixed[1] = 1
    is_fixed[3] = 1

    links = scene.objects().create("links")
    link_slot = links.geometries().create(link_mesh)[0]

    motor = LinearMotor()
    motor.apply_to(link_slot.geometry(),
                   strength=30.0,
                   motor_axis=np.array([1.0, 0.0, 0.0], dtype=np.float32),
                   motor_vel=0.0)

    prismatic = AffineBodyPrismaticJoint()
    prismatic_limit = AffineBodyPrismaticJointLimit()

    no_limit_joint = linemesh(
        np.array([[-0.75, 0.0, 0.0], [-0.45, 0.0, 0.0]], dtype=np.float32),
        np.array([[0, 1]], dtype=np.int32),
    )
    prismatic.apply_to(no_limit_joint,
                       [link_slot], [0],
                       [link_slot], [1],
                       [100.0])

    with_limit_joint = linemesh(
        np.array([[0.45, 0.0, 0.0], [0.75, 0.0, 0.0]], dtype=np.float32),
        np.array([[0, 1]], dtype=np.int32),
    )
    prismatic.apply_to(with_limit_joint,
                       [link_slot], [2],
                       [link_slot], [3],
                       [100.0])
    lower, upper = -0.08, 0.08
    prismatic_limit.apply_to(with_limit_joint, lower, upper, 350.0)

    joints = scene.objects().create("joints")
    joints.geometries().create(no_limit_joint)
    joints.geometries().create(with_limit_joint)

    def drive_linear_motor(info: Animation.UpdateInfo):
        """Drive both prismatic pairs with the same reciprocal velocity command."""
        geo = info.geo_slots()[0].geometry()
        time = info.frame() * info.dt()
        drive_vel = PRISMATIC_DRIVE_VEL_AMP * np.sin(2.0 * np.pi * PRISMATIC_DRIVE_FREQ_HZ * time)

        is_constrained = view(geo.instances().find(builtin.is_constrained))
        is_constrained[:] = 0
        is_constrained[0] = 1
        is_constrained[2] = 1

        motor_vel = view(geo.instances().find("motor_vel"))
        motor_vel[:] = 0.0
        motor_vel[0] = drive_vel
        motor_vel[2] = drive_vel

        LinearMotor.animate(geo, info.dt())

    scene.animator().insert(links, drive_linear_motor)

    world.init(scene)

    return engine, world, scene, link_slot, lower, upper


def run_gui_demo():
    """Launch Polyscope UI and step simulation interactively."""
    engine, world, scene, link_slot, lower, upper = build_scene()
    sio = SceneIO(scene)

    transforms0 = view(link_slot.geometry().transforms())
    x0_no_limit = float(transforms0[0][0, 3])
    x0_with_limit = float(transforms0[2][0, 3])

    # Keep a strong reference to engine alive during GUI loop.
    state = {"run": False, "engine": engine}

    ps.init()
    ps.set_ground_plane_mode("none")

    surf = sio.simplicial_surface()
    v = surf.positions().view()
    t = surf.triangles().topo().view()
    mesh = ps.register_surface_mesh("abd_prismatic_joint_limit", v.reshape(-1, 3), t.reshape(-1, 3))
    mesh.set_edge_width(1.0)

    def sample_values():
        """Read current joint displacement and violation metric."""
        tf = view(link_slot.geometry().transforms())
        no_limit_x = float(tf[0][0, 3] - x0_no_limit)
        with_limit_x = float(tf[2][0, 3] - x0_with_limit)
        violation = max(0.0, with_limit_x - upper) + max(0.0, lower - with_limit_x)
        return no_limit_x, with_limit_x, violation

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

        no_limit_x, with_limit_x, violation = sample_values()
        time = world.frame() * 0.01
        drive_vel = PRISMATIC_DRIVE_VEL_AMP * np.sin(2.0 * np.pi * PRISMATIC_DRIVE_FREQ_HZ * time)

        psim.Separator()
        psim.Text("Prismatic limit demo (reciprocal drive)")
        psim.Text("Left: no limit, Right: with limit")
        psim.Text(f"Frame: {world.frame()}")
        psim.Text(f"Commanded velocity: {drive_vel:+.3f} m/s")
        psim.Text(f"No limit joint value: {no_limit_x:+.4f}")
        psim.Text(f"With limit joint value: {with_limit_x:+.4f}")
        psim.Text(f"Limit range: [{lower:+.3f}, {upper:+.3f}]")
        psim.Text(f"With-limit violation: {violation:.6f}")

    ps.set_user_callback(on_update)
    ps.show()


if __name__ == "__main__":
    run_gui_demo()
