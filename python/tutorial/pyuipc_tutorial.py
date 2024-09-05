import pathlib
import os
import sys

this_file = pathlib.Path(os.path.dirname(__file__)).resolve()
module_dir = str(this_file.parent.absolute())
sys.path.append(module_dir)

from pyuipc_loader import *