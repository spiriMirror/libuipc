import numpy as np
from pyuipc_loader import pyuipc as uipc


geometry = uipc.geometry

ig = geometry.ImplicitGeometry()

vel = ig.instances().create("vel", uipc.Vector3.Zero())

print("vel:\n", vel.view())

print(uipc.Float.One())
print(uipc.Float.Zero())
print(uipc.Float.Value(3.14))

print(uipc.Vector3.Zero())
print(uipc.Vector3.Ones())
print(uipc.Vector3.Values([1, 2, 3]))
print(uipc.Vector3.Identity())
print(uipc.Vector3.UnitY())