from uipc import builtin
import argparse as ap
import os

class UIDInfo:
    def __init__(self):
        self.constitutions = builtin.ConstitutionUIDCollection.instance().to_json()
        self.implicit_geometries = builtin.ImplicitGeometryUIDCollection.instance().to_json()
    
    def sorted_uid_info(uids) -> str:
        uids = sorted(uids, key=lambda x: x['uid'])
        content:str = ''
        for u in uids:
            uid = u['uid']
            name = u['name']
            type = u['type']
            content += f'{uid} | {name} | {type}\n'
        return content
    
    def check_uid_available(self, uids, uid: int) -> bool:
        uid_map = {u['uid']:u for u in uids}
        if uid in uid_map.keys():
            u = uid_map[uid]
            name = u['name']
            type = u['type']
            # print with red color
            print(f'\033[91mUID Already Exists:\n{uid} |  {name} | {type}\033[0m')
            return False
        return True

    def print_markdown(self):
        CONTENT = f''' 
# Official Constitution

UID | Name | Type
-- | -- | --
{UIDInfo.sorted_uid_info(self.constitutions)}

# Official Implicit Geometry

UID | Name | Type
-- | -- | --
{UIDInfo.sorted_uid_info(self.implicit_geometries)}
'''
        print(CONTENT)

if __name__ == "__main__":
    uid_info = UIDInfo()
    parser = ap.ArgumentParser(description="A Tool for Libuipc UID Info")
    # '--check-constitution'
    parser.add_argument('-cc','--check-constitution', type=int, help='Check if a constitution UID is available')
    parser.add_argument('-ci','--check-implicit-geometry', type=int, help='Check if an implicit geometry UID is available')
    args = parser.parse_args()
    
    if args.check_constitution is not None:
        uid_info.check_uid_available(uid_info.constitutions, args.check_constitution)
        exit(0)

    if args.check_implicit_geometry is not None:
        uid_info.check_uid_available(uid_info.implicit_geometries, args.check_implicit_geometry)
        exit(0)
    
    uid_info.print_markdown()