import sys
import os
import json

with open(os.path.dirname(__file__) + '/../../output/build_info.json') as f:
    build_info = json.load(f)

sys.path.append(build_info['CMAKE_BINARY_DIR'] + 'Release/bin')