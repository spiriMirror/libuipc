from pyuipc_loader import pyuipc
from pyuipc import *

a = SmartObjectA()

print(view(a))

view(a).fill(1)

print(view(a))