import pytest 
import numpy as np
import math 
import os 
import pathlib
import json 
import polyscope as ps
import polyscope.imgui as psim
from pyuipc_loader import pyuipc as uipc
from pyuipc_loader import \
    Engine, World, Scene, SceneIO \
    ,Object, ContactElement, Animation

from asset import AssetDir

from pyuipc_utils.geometry import GeometrySlot \
    ,SimplicialComplex, SimplicialComplexIO \
    ,SimplicialComplexSlot \
    ,SpreadSheetIO \
    ,label_surface, label_triangle_orient, flip_inward_triangles\
    ,ground, view, linemesh, tetmesh

from pyuipc_utils.constitution import \
    StableNeoHookean, AffineBodyConstitution, ElasticModuli, \
    SoftPositionConstraint, HookeanSpring

from pyuipc_utils.gui import SceneGUI

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

run = False
@pytest.mark.example
def test_animator():
    uipc.Logger.set_level(uipc.Logger.Level.Info)

    workspace = AssetDir.output_path(__file__)

    engine = Engine("cuda", workspace)
    world = World(engine)

    config = Scene.default_config()
    print(config)

    scene = Scene(config)

    snh = StableNeoHookean()
    spc = SoftPositionConstraint()
    scene.constitution_tabular().insert(snh)
    scene.constitution_tabular().insert(spc)
    scene.contact_tabular().default_model(0.5, 1e9)
    default_element = scene.contact_tabular().default_element()

    Vs = np.array([[0, 1, 0], 
                [0, 0, 1], 
                [-np.sqrt(3)/2, 0, -0.5], 
                [np.sqrt(3)/2, 0, -0.5]
                ], dtype=np.float32)
    Ts = np.array([[0,1,2,3]])

    tet = tetmesh(Vs, Ts)
    process_surface(tet)

    moduli = ElasticModuli.youngs_poisson(1e5, 0.49)
    snh.apply_to(tet, moduli)
    spc.apply_to(tet)
    default_element.apply_to(tet)

    object = scene.objects().create("object")
    object.geometries().create(tet)

    g = ground(-1.2)
    object.geometries().create(g)

    # scripted animation
    def animation(info:Animation.UpdateInfo):
        geos:list[GeometrySlot] = info.geo_slots()
        geo:SimplicialComplex = geos[0].geometry()

        # label the constrained vertices
        is_constrained = geo.vertices().find(uipc.builtin.is_constrained)
        is_constrained_view = view(is_constrained)
        is_constrained_view[0] = 1 if info.frame() < 180 else 0

        # set the aim position
        apos = geo.vertices().find(uipc.builtin.aim_position)
        apos_view = view(apos)
        
        theta = - info.frame() * 2 * np.pi / 360
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        apos_view[0] = uipc.Vector3.Values([0, cos_t, sin_t])
        pass
    scene.animator().insert(object, animation)

    world.init(scene)

    sio = SceneIO(scene)
    sgui = SceneGUI(scene)


    ps.init()
    ps.set_ground_plane_height(-1.2)
    sgui.register()

    def on_update():
        global run
        if(psim.Button("run & stop")):
            run = not run
            
        if(run):
            world.advance()
            world.retrieve()
            sgui.update()

    ps.set_user_callback(on_update)
    ps.show()

