import sys
import os 
import json
import pathlib
import subprocess
import ctypes

this_file = os.path.dirname(__file__)
sys.path.append(this_file + '/Release/bin')
sys.path.append(this_file + '/RelWithDebInfo/bin')
