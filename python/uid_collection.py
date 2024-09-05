from pyuipc_loader import pyuipc
from pyuipc_loader import AssetDir
from pyuipc.builtin import *
import json

constitutions = ConstitutionUIDCollection.instance().to_json()
implicit_geomeries = ImplicitGeometryUIDCollection.instance().to_json()

print('constitutions:\n', constitutions)

print('implicit_geomeries:\n', implicit_geomeries)
