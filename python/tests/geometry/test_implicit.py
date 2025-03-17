import numpy as np
import uipc
import pytest 

@pytest.mark.basic 
def test_implicit():
    geometry = uipc.geometry
    ig = geometry.ImplicitGeometry()
    print(ig)
    print(ig.to_json())

    ig = geometry.ground()
    print(ig.to_json())