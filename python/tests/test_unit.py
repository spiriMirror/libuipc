import uipc 

def test_unit():
    assert uipc.unit.s == 1.0
    assert uipc.unit.m == 1.0
    assert uipc.unit.km == 1000.0
    assert uipc.unit.mm == 0.001
    assert uipc.unit.N == 1.0
    assert uipc.unit.Pa == 1.0
    assert uipc.unit.kPa == 1000.0
    assert uipc.unit.MPa == 1000000.0
    assert uipc.unit.GPa == 1000000000.0