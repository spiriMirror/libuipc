import sys
import os 

sys.path.append(os.path.dirname(__file__) + "/../../CMakeBuild/Release/bin")
print(sys.path)

from pyuipc import add 