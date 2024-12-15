import pyuipc_next as pyuipc

def test_unit():
    assert pyuipc.unit.s == 1.0
    assert pyuipc.unit.m == 1.0
    assert pyuipc.unit.km == 1000.0
    assert pyuipc.unit.mm == 0.001
    assert pyuipc.unit.N == 1.0
    assert pyuipc.unit.Pa == 1.0
    assert pyuipc.unit.kPa == 1000.0
    assert pyuipc.unit.MPa == 1000000.0
    assert pyuipc.unit.GPa == 1000000000.0