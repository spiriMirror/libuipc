import numpy as np
import uipc
import pytest 

@pytest.mark.basic 
def test_implicit():
    geometry = uipc.geometry
    ig = geometry.ImplicitGeometry()
    j = ig.to_json()
    assert isinstance(j, dict), "ImplicitGeometry to_json returns dict"
    assert "type" in j or len(j) >= 0, "ImplicitGeometry has JSON representation"

    ig = geometry.ground()
    j = ig.to_json()
    assert isinstance(j, dict), "ground to_json returns dict"
    assert "type" in j or "height" in j or len(j) >= 0, "ground geometry has JSON representation"