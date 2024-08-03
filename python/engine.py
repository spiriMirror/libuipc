from pyuipc_loader import pyuipc

engine = pyuipc.engine
world = pyuipc.world

import sys

e = engine.Engine("cuda")
w = world.World(e)
