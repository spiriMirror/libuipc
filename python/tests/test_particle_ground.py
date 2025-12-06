import numpy as np
import pytest 
import polyscope as ps
import polyscope.imgui as psim

import uipc
from uipc import Logger
from uipc import Engine, World, Scene, SceneIO
from uipc import Matrix4x4
from uipc import view
from uipc.geometry import SimplicialComplex, SimplicialComplexIO
from uipc.geometry import label_surface, label_triangle_orient, flip_inward_triangles
from uipc.geometry import ground, pointcloud
from uipc.constitution import Particle
from asset import AssetDir

from uipc.gui import SceneGUI

from asset import AssetDir

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    return sc

run = False 

@pytest.mark.example
def test_particle_ground():
    Logger.set_level(Logger.Level.Info)
    workspace = AssetDir.output_path(__file__)
    engine = Engine("cuda", workspace)
    world = World(engine)
    config = Scene.default_config()
    print(config)
    scene = Scene(config)

    pt = Particle()
    
    scene.contact_tabular().default_model(0.5, 1e9)
    default_element = scene.contact_tabular().default_element()

    n = 10
    Vs = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        Vs[i][1] = i

    Vs = Vs * 0.05 + np.array([0, 0.2, 0])
    mesh = pointcloud(Vs)
    process_surface(mesh)

    radius = 0.01
    pt.apply_to(mesh, 1e3, radius)
    default_element.apply_to(mesh)

    object = scene.objects().create("object")
    object.geometries().create(mesh)

    g = ground(0)
    object.geometries().create(g)

    world.init(scene)

    sio = SceneIO(scene)
    sgui = SceneGUI(scene)

    ps.init()
    ps.set_ground_plane_height(0)

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
