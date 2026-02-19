import pytest 
import numpy as np
import uipc
from uipc.geometry import tetmesh
from uipc import view

@pytest.mark.basic 
def test_attribute():
    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    sc = tetmesh(Vs, Ts)

    name_attr = sc.instances().create("name", "MyString")
    v = view(name_attr)
    v[0] = "hello"
    assert name_attr.view()[0] == "hello", "write then read back instance string attribute"

    sc.instances().resize(10)
    assert len(name_attr.view()) == 10, "resize(10) must grow instance attribute to 10 entries"