# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""libuipc Physics: Blender UI and native cache playback, external CUDA solver."""

from pathlib import Path
import json
import subprocess
import tempfile

import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty, EnumProperty, FloatProperty, FloatVectorProperty, IntProperty, PointerProperty, StringProperty
from bpy_extras.io_utils import ImportHelper

from . import bridge, runtime
from .protocol import MODIFIER_NAME, read_json

_pending_validation = set()


def invalidate(scene, message):
    if not scene.uipc_settings.last_bake:
        return
    for obj in scene.objects:
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if modifier and modifier.type == "MESH_CACHE":
            modifier.show_viewport = False
            modifier.show_render = False
    scene.uipc_settings.status = message


def changed(self, context):
    owner = self.id_data
    for scene in bpy.data.scenes:
        if owner == scene or (isinstance(owner, bpy.types.Object) and owner.name in scene.objects):
            invalidate(scene, "Cache is stale; bake again after changing physics settings")


class UIPCSceneSettings(bpy.types.PropertyGroup):
    python_executable: StringProperty(name="External Python", subtype="FILE_PATH",
        description="Python with pyuipc >= 0.0.28; blank uses addon preferences, then PATH")
    cache_directory: StringProperty(name="Cache Directory", subtype="DIR_PATH",
        description="Blank uses <blend name>_uipc_cache beside the saved .blend")
    substeps: IntProperty(name="Substeps", default=2, min=1, max=1000, update=changed,
        description="Solver steps per Blender frame; dt = fps_base / (fps * substeps)")
    gravity: FloatVectorProperty(name="Gravity (m/s^2)", size=3, default=(0, 0, -9.81), update=changed,
        description="Acceleration in Blender world axes, in meters per second squared")
    d_hat: FloatProperty(name="Contact Distance (m)", default=0.001, min=1e-7, soft_max=0.1, precision=6,
        update=changed, description="IPC activation distance in meters, beyond the thickness offsets")
    friction: FloatProperty(name="Friction", default=0.5, min=0, soft_max=2, update=changed,
        description="Global Coulomb friction coefficient for all contacting objects")
    resistance: FloatProperty(name="Contact Resistance (Pa)", default=1e9, min=1, soft_max=1e10,
        update=changed, description="libuipc contact model resistance")
    last_bake: StringProperty(options={"HIDDEN"})
    baked_fingerprint: StringProperty(options={"HIDDEN"})
    status: StringProperty(default="Ready", options={"SKIP_SAVE"})
    progress: FloatProperty(default=0, min=0, max=1, subtype="FACTOR", options={"SKIP_SAVE"})


class UIPCBodySettings(bpy.types.PropertyGroup):
    role: EnumProperty(name="Simulation Role", items=[
        ("NONE", "Disabled", "Not part of the libuipc simulation", 0, 0),
        ("CLOTH", "Cloth", "Baraff-Witkin membrane with discrete shell bending", 0, 1),
        ("RIGID", "Rigid Body (ABD)", "One closed, connected, consistently oriented surface", 0, 2),
        ("FEM", "Volumetric FEM", "Tetrahedral solid with Stable Neo-Hookean elasticity", 0, 4),
        ("STATIC", "Fixed Collider", "Fixed triangle surface; open surfaces are supported", 0, 3),
    ], default="NONE", update=changed)
    fixed: BoolProperty(name="Fixed Entire Object", default=False, update=changed,
        description="Fix the whole ABD instance or every FEM/cloth node, including internal nodes")
    young_modulus: FloatProperty(name="Solid Young's Modulus (Pa)", default=1e5, min=1e-6,
        soft_max=1e9, update=changed, description="3D Stable Neo-Hookean elastic modulus")
    preserve_surface: BoolProperty(name="Preserve Original Surface", default=True,
        description="Keep every original surface vertex, coordinate and triangle; only add internal nodes")
    tet_edge_length: FloatProperty(name="Target Tet Edge (m)", default=0, min=0, precision=5,
        description="Zero uses the median surface edge; controls interior resolution and optional surface refinement")
    tet_quality_passes: IntProperty(name="Quality Passes", default=4, min=0, max=100,
        description="Quality optimization passes; zero still returns a conservative, valid volume mesh")
    source_mesh: PointerProperty(type=bpy.types.Mesh, options={"HIDDEN"})
    source_groups: StringProperty(options={"HIDDEN"})
    source_role: StringProperty(default="NONE", options={"HIDDEN"})
    tet_report: StringProperty(options={"HIDDEN"})
    density: FloatProperty(name="Density (kg/m^3)", default=200, min=1e-6, soft_max=10000, update=changed)
    thickness: FloatProperty(name="Thickness Radius r (m)", default=0.001, min=1e-7, soft_max=0.05,
        precision=6, update=changed, description="One-sided collision offset r; cloth material thickness is 2*r")
    stretch: FloatProperty(name="Stretch E (Pa)", default=5e4, min=1e-6, soft_max=1e8, update=changed)
    shear: FloatProperty(name="Shear E Parameter", default=10, min=1e-6, soft_max=1e6, update=changed,
        description="Effective shear coefficient = E / (2*(1+nu)); not multiplied by thickness")
    bending: FloatProperty(name="Bending E (Pa)", default=3e4, min=0, soft_max=1e8, update=changed,
        description="Bending = E*(2*r)^3 / (12*(1-nu^2)); zero disables bending")
    poisson: FloatProperty(name="Poisson Ratio", default=0.49, min=0, max=0.499, precision=3, update=changed)
    strain_rate: FloatProperty(name="Strain Amplification", default=100, min=1e-6, soft_max=1000, update=changed)
    rigidity: FloatProperty(name="ABD Rigidity (Pa)", default=1e8, min=1, soft_max=1e10, update=changed)
    self_collision: BoolProperty(name="Self Collision", default=True, update=changed)
    pin_group: StringProperty(name="Pin Vertex Group", update=changed,
        description="Vertices meeting the weight threshold are fixed in world space")
    pin_threshold: FloatProperty(name="Pin Weight Threshold", default=0.5, min=0.0001, max=1, update=changed)


class UIPCPreferences(bpy.types.AddonPreferences):
    bl_idname = __package__
    python_executable: StringProperty(name="Default External Python", subtype="FILE_PATH",
        description="Python executable with pyuipc >= 0.0.28; individual scenes may override this")

    def draw(self, context):
        self.layout.prop(self, "python_executable")
        self.layout.label(text="Windows/Linux + NVIDIA GPU. The solver runs outside Blender.")


def _poll_timer():
    if not runtime.is_running():
        return None
    try:
        runtime.poll()
    except Exception as error:
        for scene in bpy.data.scenes:
            if scene.uipc_settings.status.startswith(("Baking", "Initializing", "Cancelling", "Preparing")):
                scene.uipc_settings.status = str(error)
        print(f"libuipc: {error}")
    for window in bpy.context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()
    return 0.2 if runtime.is_running() else None


class UIPC_OT_bake(bpy.types.Operator):
    bl_idname = "uipc.bake"
    bl_label = "Bake Simulation"
    bl_description = "Bake all enabled objects into native Blender Mesh Cache modifiers"
    blocking: BoolProperty(default=False, options={"HIDDEN", "SKIP_SAVE"})

    @classmethod
    def poll(cls, context):
        return not runtime.is_running()

    def execute(self, context):
        try:
            if self.blocking or bpy.app.background:
                runtime.bake_blocking(context.scene)
            else:
                runtime.start(context.scene)
                if not bpy.app.timers.is_registered(_poll_timer):
                    bpy.app.timers.register(_poll_timer, first_interval=0.2)
        except Exception as error:
            context.scene.uipc_settings.status = str(error)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_cancel(bpy.types.Operator):
    bl_idname = "uipc.cancel"
    bl_label = "Cancel Operation"

    def execute(self, context):
        runtime.request_cancel()
        return {"FINISHED"}


class UIPC_OT_validate(bpy.types.Operator):
    bl_idname = "uipc.validate_cache"
    bl_label = "Validate Cache"

    def execute(self, context):
        try:
            result = bridge.check_cache(context.scene)
            context.scene.uipc_settings.status = f"Cache valid: {result['frames']} frames"
            self.report({"INFO"}, context.scene.uipc_settings.status)
        except Exception as error:
            invalidate(context.scene, str(error))
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_detach(bpy.types.Operator):
    bl_idname = "uipc.detach_cache"
    bl_label = "Detach Cache"
    bl_description = "Remove libuipc cache modifiers; source meshes and cache files are retained"
    bl_options = {"UNDO"}

    @classmethod
    def poll(cls, context):
        return not runtime.is_running()

    def execute(self, context):
        bridge.detach_cache(context.scene)
        context.scene.uipc_settings.status = "Cache detached; source meshes restored"
        return {"FINISHED"}


class UIPC_OT_probe(bpy.types.Operator):
    bl_idname = "uipc.check_runtime"
    bl_label = "Check Python / CUDA"
    bl_description = "Probe the external Python package and actually initialize its CUDA backend"

    @classmethod
    def poll(cls, context):
        return not runtime.is_running()

    def execute(self, context):
        process = None
        try:
            directory = Path(tempfile.mkdtemp(prefix="libuipc_probe_"))
            result_path = directory / "probe.json"
            with (directory / "probe.log").open("wb") as log:
                command = runtime.python_command(context.scene.uipc_settings.python_executable)
                process = runtime.launch(command + [str(Path(__file__).with_name("worker.py")),
                                                   "--probe", str(result_path)], log, directory)
                try:
                    code = process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                    raise RuntimeError(f"Runtime check timed out; see {directory / 'probe.log'}")
            if code != 0 or not result_path.exists():
                raise RuntimeError(f"Python/CUDA check failed; see {directory / 'probe.log'}")
            result = read_json(result_path)
            context.scene.uipc_settings.status = f"CUDA ready: pyuipc {result['build_info']['version']} ({result['build_info']['python_abi']})"
            self.report({"INFO"}, context.scene.uipc_settings.status)
        except Exception as error:
            context.scene.uipc_settings.status = str(error)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_demo(bpy.types.Operator):
    bl_idname = "uipc.create_demo"
    bl_label = "Create Example Scene"
    bl_description = "Create a separate cloth/ABD/contact example scene"

    @classmethod
    def poll(cls, context):
        return not runtime.is_running()

    def execute(self, context):
        from .demo import create_demo
        scene = create_demo()
        if context.window:
            context.window.scene = scene
        return {"FINISHED"}


class UIPC_OT_generate_volume(bpy.types.Operator):
    bl_idname = "uipc.generate_volume"
    bl_label = "Generate Tetrahedra"
    bl_description = "Generate a volume mesh with libuipc's native boundary-conforming tetrahedralizer"
    blocking: BoolProperty(default=False, options={"HIDDEN", "SKIP_SAVE"})

    @classmethod
    def poll(cls, context):
        return not runtime.is_running() and context.object is not None and context.object.type == "MESH"

    def execute(self, context):
        try:
            if self.blocking or bpy.app.background:
                runtime.bake_blocking(context.scene, volume_object=context.object)
            else:
                runtime.start(context.scene, volume_object=context.object)
                if not bpy.app.timers.is_registered(_poll_timer):
                    bpy.app.timers.register(_poll_timer, first_interval=0.2)
        except Exception as error:
            context.scene.uipc_settings.status = str(error)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_import_volume(bpy.types.Operator, ImportHelper):
    bl_idname = "uipc.import_volume"
    bl_label = "Import FEM Mesh (.msh)"
    filename_ext = ".msh"
    filter_glob: StringProperty(default="*.msh", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return not runtime.is_running()

    def execute(self, context):
        try:
            if bpy.app.background:
                runtime.bake_blocking(context.scene, volume_file=self.filepath)
            else:
                runtime.start(context.scene, volume_file=self.filepath)
                if not bpy.app.timers.is_registered(_poll_timer):
                    bpy.app.timers.register(_poll_timer, first_interval=0.2)
        except Exception as error:
            context.scene.uipc_settings.status = str(error)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_restore_surface(bpy.types.Operator):
    bl_idname = "uipc.restore_surface"
    bl_label = "Restore Original Surface"
    bl_description = "Restore the source mesh and its original vertex-group definitions"
    bl_options = {"UNDO"}

    @classmethod
    def poll(cls, context):
        return (not runtime.is_running() and context.object is not None
                and context.object.uipc_body.source_mesh is not None)

    def execute(self, context):
        bridge.restore_surface(context.object)
        context.scene.uipc_settings.status = "Original surface restored"
        return {"FINISHED"}


class UIPC_PT_scene(bpy.types.Panel):
    bl_label = "libuipc Physics"
    bl_idname = "UIPC_PT_scene"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "libuipc"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.uipc_settings
        column = layout.column()
        column.enabled = not runtime.is_running()
        column.prop(settings, "python_executable")
        column.operator("uipc.check_runtime")
        column.prop(settings, "cache_directory")
        row = column.row(align=True)
        row.prop(context.scene, "frame_start", text="Start")
        row.prop(context.scene, "frame_end", text="End")
        column.label(text=f"{context.scene.render.fps / context.scene.render.fps_base:g} FPS; {context.scene.unit_settings.scale_length:g} m / unit")
        column.prop(settings, "substeps")
        column.prop(settings, "gravity")
        column.prop(settings, "d_hat")
        column.prop(settings, "friction")
        column.prop(settings, "resistance")
        layout.operator("uipc.cancel" if runtime.is_running() else "uipc.bake")
        layout.label(text=settings.status[:90])
        if runtime.is_running():
            layout.progress(factor=settings.progress, type="BAR", text="Bake Progress")
        row = layout.row(align=True)
        row.enabled = not runtime.is_running()
        row.operator("uipc.validate_cache")
        row.operator("uipc.detach_cache")
        layout.operator("uipc.create_demo")
        layout.operator("uipc.import_volume")


class UIPC_PT_body(bpy.types.Panel):
    bl_label = "Object Physics"
    bl_idname = "UIPC_PT_body"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "libuipc"

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == "MESH"

    def draw(self, context):
        layout = self.layout
        layout.enabled = not runtime.is_running()
        body = context.object.uipc_body
        layout.prop(body, "role")
        if body.role == "NONE":
            return
        layout.label(text="Simulation uses the base mesh")
        layout.prop(body, "thickness")
        if body.role != "STATIC":
            layout.prop(body, "fixed")
            layout.prop(body, "density")
        if body.role == "RIGID":
            layout.prop(body, "rigidity")
        if body.role == "CLOTH":
            layout.label(text="Material thickness = 2 * r")
            for name in ("stretch", "shear", "bending", "poisson", "strain_rate", "self_collision"):
                layout.prop(body, name)
        if body.role == "FEM":
            cells = context.object.data.get("uipc_tetrahedra")
            if cells is None:
                layout.prop(body, "preserve_surface")
                layout.prop(body, "tet_edge_length")
                layout.prop(body, "tet_quality_passes")
                layout.operator("uipc.generate_volume")
            else:
                layout.label(text=f"{len(context.object.data.vertices)} nodes, {len(cells)//4} tetrahedra")
                try:
                    report = json.loads(body.tet_report or "{}")
                except ValueError:
                    report = {}
                if "min_quality" in report:
                    layout.label(text=f"Minimum cell quality: {report['min_quality']:.3f}")
                if report.get("preserve_surface"):
                    layout.label(text="Original surface preserved")
                layout.operator("uipc.restore_surface")
            layout.prop(body, "young_modulus")
            layout.prop(body, "poisson")
            layout.prop(body, "self_collision")
        if body.role in ("CLOTH", "FEM"):
            pins = layout.column()
            pins.enabled = not body.fixed
            pins.prop_search(body, "pin_group", context.object, "vertex_groups")
            if body.pin_group:
                pins.prop(body, "pin_threshold")


def _validate_pending():
    names = list(_pending_validation)
    _pending_validation.clear()
    if runtime.is_running():
        return None  # Completion revalidates the full scene before attaching output.
    for name in names:
        scene = bpy.data.scenes.get(name)
        if scene and scene.uipc_settings.last_bake:
            try:
                bridge.check_cache(scene)
            except Exception as error:
                invalidate(scene, str(error))
    return None


@persistent
def _depsgraph_updated(scene, depsgraph):
    if not scene.uipc_settings.last_bake or runtime.is_running():
        return
    for update in depsgraph.updates:
        if isinstance(update.id, bpy.types.Mesh) or (isinstance(update.id, bpy.types.Object) and update.is_updated_transform):
            _pending_validation.add(scene.name)
            if not bpy.app.timers.is_registered(_validate_pending):
                bpy.app.timers.register(_validate_pending, first_interval=0.5)
            break


@persistent
def _load_pre(_):
    runtime.stop()
    _pending_validation.clear()


@persistent
def _load_post(_):
    for scene in bpy.data.scenes:
        if scene.uipc_settings.last_bake:
            _pending_validation.add(scene.name)
    if _pending_validation and not bpy.app.timers.is_registered(_validate_pending):
        bpy.app.timers.register(_validate_pending, first_interval=0.1)


CLASSES = (UIPCSceneSettings, UIPCBodySettings, UIPCPreferences, UIPC_OT_bake, UIPC_OT_cancel,
           UIPC_OT_validate, UIPC_OT_detach, UIPC_OT_probe, UIPC_OT_demo,
           UIPC_OT_generate_volume, UIPC_OT_import_volume, UIPC_OT_restore_surface,
           UIPC_PT_scene, UIPC_PT_body)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.uipc_settings = PointerProperty(type=UIPCSceneSettings)
    bpy.types.Object.uipc_body = PointerProperty(type=UIPCBodySettings)
    bpy.app.handlers.load_pre.append(_load_pre)
    bpy.app.handlers.load_post.append(_load_post)
    bpy.app.handlers.depsgraph_update_post.append(_depsgraph_updated)


def unregister():
    runtime.stop()
    _pending_validation.clear()
    for timer in (_poll_timer, _validate_pending):
        if bpy.app.timers.is_registered(timer):
            bpy.app.timers.unregister(timer)
    for handlers, function in ((bpy.app.handlers.load_pre, _load_pre),
                               (bpy.app.handlers.load_post, _load_post),
                               (bpy.app.handlers.depsgraph_update_post, _depsgraph_updated)):
        if function in handlers:
            handlers.remove(function)
    del bpy.types.Object.uipc_body
    del bpy.types.Scene.uipc_settings
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
