from project_dir import project_dir
import os
import shutil

include_dir = project_dir() / 'include'
src_dir = project_dir() / 'src'

def detect_0kb_files():
    ps = []
    for root, dirs, files in os.walk(include_dir):
        for file in files:
            path = os.path.join(root, file)
            if os.path.getsize(path) == 0:
                print(f'{path}')
                ps.append(path)
                
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            path = os.path.join(root, file)
            if os.path.getsize(path) == 0:
                print(f'{path}')
                ps.append(path)
    return ps

if __name__ == "__main__":
    detect_0kb_files()
    
    # delete 0kb files
    # for p in detect_0kb_files():
    #     os.remove(p)