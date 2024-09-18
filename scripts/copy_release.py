import os 
import shutil

cwd = os.getcwd()
current_dir = os.path.dirname(os.path.realpath(__file__))

release_name = "RelWithDebInfo"
source_dir = os.path.join(cwd, release_name)
target_dir = os.path.join(current_dir, "../python/src/pyuipc/", release_name)

# copy recursive
shutil.copytree(source_dir, target_dir)