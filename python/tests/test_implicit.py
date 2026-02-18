import numpy as np
import uipc
import pytest 

@pytest.mark.basic 
def test_implicit():
    geometry = uipc.geometry
    ig = geometry.ImplicitGeometry()
    j = ig.to_json()
    assert isinstance(j, dict), "ImplicitGeometry to_json returns dict"
    assert "type" in j or "meta" in j or not j, "ImplicitGeometry has JSON representation (type, meta, or empty)"

    ig = geometry.ground()
    j = ig.to_json()
    assert isinstance(j, dict), "ground to_json returns dict"
    assert "type" in j or "height" in j or "meta" in j, "ground geometry has JSON representation"