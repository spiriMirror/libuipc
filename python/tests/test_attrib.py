import pytest 
import numpy as np
import uipc
from uipc.geometry import tetmesh
from uipc import view

@pytest.mark.basic 
def test_attrib():
    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    sc = tetmesh(Vs, Ts)

    name_attr = sc.instances().create("name", "MyString")
    print("name_attr:\n", name_attr.view())
    # get python attribute of name_attr
    v = view(name_attr)
    v[0] = "hello"
    print("name_attr:\n", name_attr.view())

    sc.instances().resize(10)
    print("name_attr:\n", name_attr.view())