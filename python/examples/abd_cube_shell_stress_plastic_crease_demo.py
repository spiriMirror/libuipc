"""
ABD cube -> shell stress-plastic crease demo.

This demo mirrors the plastic crease setup, but uses the stress-based
ECI shell bending model `StressPlasticDiscreteShellBending`.
"""

import os
import sys
import numpy as np

try:
    import polyscope as ps
    import polyscope.imgui as psim
except ModuleNotFoundError as exc:
    raise SystemExit(
        "This example requires `polyscope`. Install it with `pip install polyscope`."
    ) from exc

try:
    from uipc import Logger, Matrix4x4, Engine, World, Scene, SceneIO, Animation, view
    from uipc.geometry import (
        SimplicialComplex,
        SimplicialComplexIO,
        trimesh,
        ground,
        label_surface,
        label_triangle_orient,
        flip_inward_triangles,
    )
    from uipc.constitution import (
        AffineBodyConstitution,
        SoftTransformConstraint,
        SoftPositionConstraint,
        NeoHookeanShell,
        StressPlasticDiscreteShellBending,
        ElasticModuli2D,
    )
    from uipc import builtin
except ImportError as exc:
    raise SystemExit(
        "This example requires the libuipc Python bindings (`uipc._native.pyuipc`). "
        "Build/install the Python package before running it."
    ) from exc

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "tests"))
from asset import AssetDir


SHEET_RESOLUTION = 21
SHEET_SIZE = 1.6
SHEET_THICKNESS = 0.0025
SHELL_DENSITY = 200.0
SHELL_YOUNG = 8.0e4
SHELL_POISSON = 0.35
BENDING_STIFFNESS = 4.0e3
YIELD_STRESS = 9.6e2
HARDENING_MODULUS = 0.0

CUBE_SCALE = 0.6
CUBE_Y_HIGH = 1.35
CUBE_Y_LOW = 0.33
GROUND_Y = -0.01

GATHER_FRAMES = 120
GATHER_HOLD_FRAMES = 40
PRESS_FRAMES = 180
PRESS_HOLD_FRAMES = 220
RELEASE_FRAMES = 160
SETTLE_FRAMES = 180
TOTAL_FRAMES = (
    GATHER_FRAMES
    + GATHER_HOLD_FRAMES
    + PRESS_FRAMES
    + PRESS_HOLD_FRAMES
    + RELEASE_FRAMES
    + SETTLE_FRAMES
)


def process_closed_surface(sc: SimplicialComplex) -> SimplicialComplex:
    label_surface(sc)
    label_triangle_orient(sc)
    return flip_inward_triangles(sc)


def make_sheet_mesh(resolution: int, size: float) -> SimplicialComplex:
    xs = np.linspace(-0.5 * size, 0.5 * size, resolution)
    zs = np.linspace(-0.5 * size, 0.5 * size, resolution)

    vertices = []
    triangles = []

    for z in zs:
        for x in xs:
            vertices.append([x, 0.0, z])

    for j in range(resolution - 1):
        for i in range(resolution - 1):
            v00 = j * resolution + i
            v10 = v00 + 1
            v01 = v00 + resolution
            v11 = v01 + 1
            triangles.append([v00, v10, v11])
            triangles.append([v00, v11, v01])

    sheet = trimesh(np.asarray(vertices, dtype=np.float64),
                    np.asarray(triangles, dtype=np.int32))
    label_surface(sheet)
    return sheet


def cube_transform(center_y: float) -> Matrix4x4:
    transform = Matrix4x4.Identity()
    transform[0:3, 3] = np.array([0.0, center_y, 0.0], dtype=np.float64)
    return transform


def smooth_lerp(a: float, b: float, t: float) -> float:
    t = np.clip(t, 0.0, 1.0)
    s = 0.5 - 0.5 * np.cos(np.pi * t)
    return a + (b - a) * s


def motion_schedule(frame: int) -> tuple[float, float, str]:
    if frame < GATHER_FRAMES:
        arc_alpha = smooth_lerp(0.0, 1.0, frame / max(GATHER_FRAMES - 1, 1))
        return arc_alpha, CUBE_Y_HIGH, "gather"

    frame -= GATHER_FRAMES
    if frame < GATHER_HOLD_FRAMES:
        return 1.0, CUBE_Y_HIGH, "gather-hold"

    frame -= GATHER_HOLD_FRAMES
    if frame < PRESS_FRAMES:
        cube_y = smooth_lerp(CUBE_Y_HIGH, CUBE_Y_LOW, frame / max(PRESS_FRAMES - 1, 1))
        return 1.0, cube_y, "press"

    frame -= PRESS_FRAMES
    if frame < PRESS_HOLD_FRAMES:
        return 1.0, CUBE_Y_LOW, "press-hold"

    frame -= PRESS_HOLD_FRAMES
    if frame < RELEASE_FRAMES:
        cube_y = smooth_lerp(CUBE_Y_LOW, CUBE_Y_HIGH, frame / max(RELEASE_FRAMES - 1, 1))
        return 1.0, cube_y, "release"

    return 1.0, CUBE_Y_HIGH, "inspect"


def upper_arc_point(start: np.ndarray, end: np.ndarray, alpha: float) -> np.ndarray:
    alpha = np.clip(alpha, 0.0, 1.0)
    chord = end - start
    chord_len = np.linalg.norm(chord)
    if chord_len < 1.0e-8:
        return end.copy()

    center = 0.5 * (start + end)
    radius = 0.5 * chord_len
    u = (start - center) / radius
    v = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    theta = np.pi * alpha
    return center + radius * (np.cos(theta) * u + np.sin(theta) * v)


def build_demo():
    Logger.set_level(Logger.Level.Warn)

    workspace = AssetDir.output_path(__file__)
    engine = Engine("cuda", workspace)
    world = World(engine)

    config = Scene.default_config()
    config["dt"] = 0.01
    config["gravity"] = [[0.0], [0.0], [0.0]]
    config["contact"]["enable"] = True
    config["contact"]["friction"]["enable"] = False
    config["line_search"]["max_iter"] = 12
    config["linear_system"]["tol_rate"] = 1.0e-3
    scene = Scene(config)

    scene.contact_tabular().default_model(0.2, 1.0e9)
    default_contact = scene.contact_tabular().default_element()

    sheet_object = scene.objects().create("sheet")
    cube_object = scene.objects().create("press_cube")

    shell = NeoHookeanShell()
    stress_plastic_bending = StressPlasticDiscreteShellBending()
    abd = AffineBodyConstitution()
    stc = SoftTransformConstraint()
    spc = SoftPositionConstraint()

    sheet = make_sheet_mesh(SHEET_RESOLUTION, SHEET_SIZE)
    moduli = ElasticModuli2D.youngs_poisson(SHELL_YOUNG, SHELL_POISSON)
    shell.apply_to(sheet, moduli, SHELL_DENSITY, SHEET_THICKNESS)
    try:
        stress_plastic_bending.apply_to(
            sheet,
            BENDING_STIFFNESS,
            YIELD_STRESS,
            HARDENING_MODULUS,
        )
    except TypeError as exc:
        raise SystemExit(
            "Your installed pyuipc bindings are older than this branch. "
            "Rebuild/reinstall the Python bindings so "
            "StressPlasticDiscreteShellBending is available and accepts "
            "(sc, bending_stiffness, yield_stress, hardening_modulus)."
        ) from exc
    spc.apply_to(sheet, 150.0)
    default_contact.apply_to(sheet)

    sheet_rest_positions = np.array(view(sheet.positions()), copy=True).reshape(-1, 3)
    sheet_slot = sheet_object.geometries().create(sheet)[0]

    pre = Matrix4x4.Identity()
    pre[0, 0] = CUBE_SCALE
    pre[1, 1] = CUBE_SCALE
    pre[2, 2] = CUBE_SCALE

    io = SimplicialComplexIO(pre)
    cube = io.read(f"{AssetDir.tetmesh_path()}/cube.msh")
    cube = process_closed_surface(cube)
    abd.apply_to(cube, 2.0e7)
    stc.apply_to(cube, np.array([3600.0, 120.0], dtype=np.float64))
    default_contact.apply_to(cube)

    view(cube.transforms())[0] = cube_transform(CUBE_Y_HIGH)
    cube_object.geometries().create(cube)

    ground_object = scene.objects().create("ground")
    ground_object.geometries().create(ground(GROUND_Y))

    corner_a = 0
    corner_b = SHEET_RESOLUTION * SHEET_RESOLUTION - 1
    corner_c = SHEET_RESOLUTION - 1
    corner_d = SHEET_RESOLUTION * (SHEET_RESOLUTION - 1)
    rest_a = sheet_rest_positions[corner_a].copy()
    rest_b = sheet_rest_positions[corner_b].copy()
    rest_c = sheet_rest_positions[corner_c].copy()
    rest_d = sheet_rest_positions[corner_d].copy()

    def animate_sheet(info: Animation.UpdateInfo):
        geo = info.geo_slots()[0].geometry()
        is_constrained = view(geo.vertices().find(builtin.is_constrained))
        aim_position = view(geo.vertices().find(builtin.aim_position))

        is_constrained[:] = 0
        is_constrained[corner_a] = 1
        is_constrained[corner_b] = 1
        is_constrained[corner_c] = 1
        is_constrained[corner_d] = 1

        arc_alpha, _, _ = motion_schedule(max(info.frame() - 1, 0))
        aim_position[corner_a] = rest_a.reshape(3, 1)
        aim_position[corner_b] = rest_b.reshape(3, 1)
        aim_position[corner_c] = rest_c.reshape(3, 1)
        aim_position[corner_d] = upper_arc_point(rest_d, rest_c, arc_alpha).reshape(3, 1)

    def animate_cube(info: Animation.UpdateInfo):
        geo = info.geo_slots()[0].geometry()
        is_constrained = view(geo.instances().find(builtin.is_constrained))
        aim_transform = view(geo.instances().find(builtin.aim_transform))

        is_constrained[:] = 0
        is_constrained[0] = 1

        _, center_y, _ = motion_schedule(max(info.frame() - 1, 0))
        aim_transform[0] = cube_transform(center_y)

    scene.animator().insert(sheet_object, animate_sheet)
    scene.animator().insert(cube_object, animate_cube)

    world.init(scene)

    return {
        "engine": engine,
        "world": world,
        "scene_io": SceneIO(scene),
        "sheet_slot": sheet_slot,
        "sheet_rest_positions": sheet_rest_positions,
        "moving_corner": corner_d,
        "target_corner": corner_c,
    }


def run_demo():
    state = build_demo()
    world = state["world"]
    scene_io = state["scene_io"]
    sheet_slot = state["sheet_slot"]
    sheet_rest_positions = state["sheet_rest_positions"]
    moving_corner = state["moving_corner"]
    target_corner = state["target_corner"]

    ps.init()
    ps.set_ground_plane_mode("none")

    surface = scene_io.simplicial_surface()
    mesh = ps.register_surface_mesh("abd_cube_shell_stress_plastic_crease",
                                    surface.positions().view().reshape(-1, 3),
                                    surface.triangles().topo().view().reshape(-1, 3))
    mesh.set_edge_width(1.0)

    ui_state = {"run": True}

    def update_visual_mesh():
        merged = scene_io.simplicial_surface()
        mesh.update_vertex_positions(merged.positions().view().reshape(-1, 3))

    def sheet_metrics() -> tuple[float, float, float]:
        current_positions = np.array(view(sheet_slot.geometry().positions()), copy=True).reshape(-1, 3)
        displacement = current_positions - sheet_rest_positions
        max_disp = float(np.linalg.norm(displacement, axis=1).max())
        crease_depth = float(np.max(sheet_rest_positions[:, 1] - current_positions[:, 1]))
        corner_gap = float(np.linalg.norm(current_positions[moving_corner] - current_positions[target_corner]))
        return max_disp, crease_depth, corner_gap

    def advance_one_frame():
        if world.frame() >= TOTAL_FRAMES:
            ui_state["run"] = False
            return

        world.advance()
        if not world.is_valid():
            ui_state["run"] = False
            return

        world.retrieve()
        update_visual_mesh()

    def on_update():
        if psim.Button("run / pause"):
            ui_state["run"] = not ui_state["run"]

        psim.SameLine()
        if psim.Button("step"):
            advance_one_frame()

        if ui_state["run"]:
            advance_one_frame()

        frame = min(world.frame(), TOTAL_FRAMES)
        arc_alpha, cube_y, phase = motion_schedule(frame)
        max_disp, crease_depth, corner_gap = sheet_metrics()

        psim.Separator()
        psim.Text("ECI shell bending plasticity with stress-based yielding")
        psim.Text(f"Frame: {world.frame()} / {TOTAL_FRAMES}")
        psim.Text(f"Phase: {phase}")
        psim.Text(f"Arc progress: {arc_alpha:.3f}")
        psim.Text(f"Cube target Y: {cube_y:+.3f}")
        psim.Text(f"Yield stress: {YIELD_STRESS:.3f}")
        psim.Text(f"Hardening modulus: {HARDENING_MODULUS:.3f}")
        psim.Text(f"Max sheet displacement: {max_disp:.4f}")
        psim.Text(f"Residual crease depth: {crease_depth:.4f}")
        psim.Text(f"Diagonal corner gap: {corner_gap:.4f}")

    ps.set_user_callback(on_update)
    ps.show()


if __name__ == "__main__":
    run_demo()
