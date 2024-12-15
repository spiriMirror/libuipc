import pytest 
import numpy as np
import uipc 

@pytest.mark.basic 
def test_attrib():
    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])
    Ts = np.array([[0,1,2,3]])
    sc = uipc.geometry.tetmesh(Vs, Ts)
    attr_k = "name"
    attr_v = "MyString"
    name_attr = sc.instances().create(attr_k, attr_v)
    assert len(name_attr.view()) == 1
    assert name_attr.view()[0] == attr_v
    # get python attribute of name_attr
    v = uipc.view(name_attr)
    v[0] = "hello"
    assert name_attr.view()[0] == "hello"
    sc.instances().resize(5)
    assert len(name_attr.view()) == 5
    for i in range(5):
        if (i == 0):
            assert name_attr.view()[i] == "hello"
        else:
            assert name_attr.view()[i] == attr_v