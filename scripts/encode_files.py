import os
import pathlib
import shutil
# local
red         = "\033[131m"
off         = "\033[00m"

subfix = ['.cpp','.hpp','.h','.c','.cu','.cuh']

if(os.system("iconv --version") != 0):
    print("[iconv] not found, please install it first")
    exit(1)

# if arg1 is not empty, then use it as root directory
if(len(os.sys.argv) > 1):
    os.chdir(os.sys.argv[1])
path = pathlib.Path(".").absolute()
print(f"traverse root directory:{path}")
print("enter y to continue, other to exit")

if(input() != "y" and input() != "Y"):
    exit(0)

fail_list = []
for root, dirs, files in os.walk("."):
    path = root.split(os.sep)
    print((len(path) - 1) * '-', os.path.basename(root))
    for file in files:
        print(len(path) * '-', file)
        s = pathlib.Path(file).suffix
        if(s in subfix):
            fullname = os.path.join(root, file)
            fullnametemp = fullname + ".tmp"
            if(os.system("iconv -t UTF-8 " + fullname + " > " + fullnametemp) == 0):
                shutil.copy(fullnametemp, fullname)
            else:
                fail_list.append( pathlib.Path(fullname).absolute())
            os.remove(fullnametemp)

for fail in fail_list:
    print(red, "fail to convert:", fail, off)

if(len(fail_list) == 0):
    print("all success!")