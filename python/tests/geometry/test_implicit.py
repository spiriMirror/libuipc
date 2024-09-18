import numpy as np
from pyuipc_loader import pyuipc
import pytest 

@pytest.mark.basic 
def test_implicit():
    geometry = pyuipc.geometry
    ig = geometry.ImplicitGeometry()
    print(ig)
    print(ig.to_json())

    ig = geometry.ground()
    print(ig.to_json())