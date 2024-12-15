from pyuipc_next.geometry import TetMesh 
import numpy as np 

def test_geometry_attr():
    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    # sc = pyuipc.geometry.tetmesh(Vs, Ts)
    mesh = TetMesh(Vs, Ts)
    attr = mesh.create_attr("name", "MyString")
    print("name_attr:\n", attr.view())
    v = mesh["name"]
    v[0] = "hello"
    print("name_attr:\n", attr.view())
    mesh.resize(10)
    print("name_attr:\n", attr.view())