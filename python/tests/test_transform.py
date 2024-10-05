import pytest 
from pyuipc_loader import pyuipc as uipc 
import numpy as np
import polyscope as ps
from pyuipc_loader import pyuipc
from pyuipc import Logger
from pyuipc import Engine, World, Scene, SceneIO
from pyuipc import Matrix4x4
from pyuipc.geometry import SimplicialComplex, SimplicialComplexIO
from pyuipc.geometry import label_surface, label_triangle_orient, flip_inward_triangles
from pyuipc.geometry import ground, view, tetmesh, merge, extract_surface
from pyuipc.constitution import StableNeoHookean, ElasticModuli
from asset import AssetDir

@pytest.mark.current 
def test_transform():
    Transform = uipc.Transform
    AngleAxis = uipc.AngleAxis
    Vector3 = uipc.Vector3

    # create a tetrahedron
    Vs = np.array([[0,1,0],
                [0,0,1],
                [-np.sqrt(3)/2, 0, -0.5],
                [np.sqrt(3)/2, 0, -0.5]])

    Ts = np.array([[0,1,2,3]])

    T = Transform.Identity()
    A = AngleAxis(np.pi/4, Vector3.UnitZ())
    T.rotate(A)

    print(Vs)
    sc1 = tetmesh(Vs, Ts)
    pos1_view = view(sc1.positions())
    T.apply_to(pos1_view)

    T = Transform.Identity()
    T.translate(Vector3.UnitY() * 2)
    T.scale(1.2)
    sc2 = tetmesh(Vs, Ts)
    pos2_view = view(sc2.positions())
    T.apply_to(pos2_view)

    sc = merge([sc1, sc2])
    label_surface(sc)
    surf = extract_surface(sc)

    ps.init()
    ps.register_surface_mesh("tetrahedron", surf.positions().view().reshape(-1,3), surf.triangles().topo().view().reshape(-1,3))
    ps.register_curve_network("tetrahedron_edges", surf.positions().view().reshape(-1,3), surf.edges().topo().view().reshape(-1,2))
    ps.register_point_cloud("tetrahedron_vertices", surf.positions().view().reshape(-1,3))
    ps.show()


