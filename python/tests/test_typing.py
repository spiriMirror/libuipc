import pytest 
import numpy as np 
import uipc
from uipc import Float, Vector3

@pytest.mark.typing 
def test_typings():
    assert type(Float.One()) == float
    assert Float.One() == 1.0
    assert Float.Zero() == 0.0
    assert Float.Value(3.14) == 3.14
    assert type(Vector3.Zero()) == np.ndarray
    assert np.all(Vector3.Zero() == np.zeros(3))
    assert np.all(Vector3.Ones() == np.ones(3))
    assert np.all(Vector3.Values([1, 2, 3]) == np.array([1, 2, 3]).reshape(3, 1))
    assert np.all(Vector3.Identity() == np.array([1, 0, 0]).reshape(3, 1))
    assert np.all(Vector3.UnitY() == np.array([0, 1, 0]).reshape(3, 1))