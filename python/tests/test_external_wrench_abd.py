# -*- coding: utf-8 -*-
# @file test_external_wrench_abd.py
# @brief Test for ExternalWrenchABD constitution
# @author User-defined
# @date 2025-01-19

import pytest
import numpy as np
import polyscope as ps
import polyscope.imgui as psim
from uipc import Logger
from uipc import Matrix4x4, Vector3
from uipc import Engine, World, Scene, SceneIO, Animation
from uipc.backend import SceneVisitor
from uipc.geometry import SimplicialComplex, SimplicialComplexIO
from uipc.geometry import label_surface, label_triangle_orient, flip_inward_triangles
from uipc.geometry import ground
from uipc import view
from uipc.constitution import AffineBodyConstitution, ExternalWrenchABD
from asset import AssetDir


def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc


run = False


@pytest.mark.example
def test_external_wrench_abd():
    Logger.set_level(Logger.Level.Info)
    workspace = AssetDir.output_path(__file__)
    engine = Engine("cuda", workspace)
    world = World(engine)
    config = Scene.default_config()
    config['gravity'] = [[0], [0], [0]]
    print(config)
    scene = Scene(config)

    # Create constitutions
    abd = AffineBodyConstitution()
    ext_wrench = ExternalWrenchABD()

    scene.constitution_tabular().insert(abd)
    scene.constitution_tabular().insert(ext_wrench)

    # Setup contact
    scene.contact_tabular().default_model(0.5, 1e9)
    default_element = scene.contact_tabular().default_element()

    # Load cube mesh
    pre_trans = Matrix4x4.Identity()
    pre_trans[0, 0] = 0.2
    pre_trans[1, 1] = 0.2
    pre_trans[2, 2] = 0.2

    io = SimplicialComplexIO(pre_trans)
    cube = io.read(f'{AssetDir.tetmesh_path()}/cube.msh')
    cube = process_surface(cube)

    # Apply constitutions
    abd.apply_to(cube, 1e8)  # stiffness

    # Apply external wrench - initially zero, will be controlled by animator
    initial_wrench = np.zeros(12)  # Vector12: [fx, fy, fz, dS/dt (9 components)]
    ext_wrench.apply_to(cube, initial_wrench)

    default_element.apply_to(cube)

    # Create object with single cube (to see both orbital and spinning motion clearly)
    cube_object = scene.objects().create("cube")

    trans = Matrix4x4.Identity()
    trans[0:3, 3] = np.array([0, 0.5, 0])  # Position at y=0.5
    view(cube.transforms())[0] = trans
    cube_object.geometries().create(cube)

    # Add animator to control wrench for combined orbital and spinning motion
    def animate_rotating_force(info: Animation.UpdateInfo):
        """
        Apply a 12D wrench with both translational force and rotational component.
        The cube will orbit around the origin while also spinning around its own Y axis.
        """
        print(f"========== ANIMATOR CALLED: Frame {info.frame()} ==========")

        geo_slots = info.geo_slots()
        time = info.dt() * info.frame()

        print(f"  Time: {time:.2f}s, dt: {info.dt():.4f}s, num_geos: {len(geo_slots)}")

        # Rotation parameters (reduced for stability)
        orbit_speed = 0.2  # rad/s - speed of orbital motion
        spin_speed = 0.1   # rad/s - speed of spinning motion
        force_magnitude = 0.1  # N (reduced for stability)

        # Calculate rotating force direction (in XZ plane for orbital motion)
        orbit_angle = orbit_speed * time
        force_direction = np.array([np.cos(orbit_angle), 0.0, np.sin(orbit_angle)])
        force_3d = force_direction * force_magnitude

        # Calculate shape velocity derivative for spinning (rotation around Y axis)
        # For ABD, shape matrix S is 3x3, dS/dt represents rate of change
        # For rotation around Y: dS/dt = skew([0, ω, 0]) * S
        # Antisymmetric contribution for rotational motion
        omega_y = spin_speed * 0.1  # Much smaller scale for stability
        shape_vel_derivative = np.array([
            0.0, 0.0, omega_y,   # row 1: [dS11, dS12, dS13]
            0.0, 0.0, 0.0,       # row 2: [dS21, dS22, dS23]
            -omega_y, 0.0, 0.0   # row 3: [dS31, dS32, dS33]
        ])

        # Apply to all geometries (cubes)
        for i, geo_slot in enumerate(geo_slots):
            geo = geo_slot.geometry()
            if geo is None:
                continue

            # Find external_wrench attribute
            wrench_attr = geo.instances().find("external_wrench")

            if wrench_attr is None:
                continue

            # Create Vector12 wrench:
            # [0:3] = linear force (orbital motion)
            # [3:12] = shape velocity derivative (spinning motion)
            wrench_v12 = np.zeros(12)
            wrench_v12[0:3] = force_3d
            wrench_v12[3:12] = shape_vel_derivative

            # Update the wrench (reshape to match attribute shape)
            view(wrench_attr)[:] = wrench_v12.reshape(-1, 1)

            # Debug output every 50 frames
            if info.frame() % 50 == 0 and i == 0:
                print(f"Frame {info.frame()}: Force = [{force_3d[0]:.2f}, {force_3d[1]:.2f}, {force_3d[2]:.2f}] N, Spin = {omega_y:.2f}")

    print("Registering animator...")
    scene.animator().insert(cube_object, animate_rotating_force)
    print("Animator registered successfully")

    sio = SceneIO(scene)
    world.init(scene)

    use_gui = True
    if use_gui:
        ps.init()
        ps.set_ground_plane_mode('none')
        s = sio.simplicial_surface()
        v = s.positions().view()
        t = s.triangles().topo().view()
        mesh = ps.register_surface_mesh('obj', v.reshape(-1, 3), t.reshape(-1, 3))
        mesh.set_edge_width(1.0)

        frame_count = [0]  # Mutable counter for frame tracking

        def on_update():
            global run
            if psim.Button("run & stop"):
                run = not run

            if run:
                world.advance()
                world.retrieve()
                frame_count[0] += 1

                # Update visualization
                s = sio.simplicial_surface()
                v = s.positions().view()
                mesh.update_vertex_positions(v.reshape(-1, 3))

        ps.set_user_callback(on_update)
        ps.show()
    else:
        # No GUI mode
        sio.write_surface(f'{workspace}/scene_surface{0}.obj')
        world.recover()

        for frame in range(100):
            world.advance()
            world.retrieve()
            sio.write_surface(f'{workspace}/scene_surface{world.frame()}.obj')
            world.dump()

            if frame % 10 == 0:
                print(f"Frame {world.frame()} completed")


if __name__ == "__main__":
    test_external_wrench_abd()
