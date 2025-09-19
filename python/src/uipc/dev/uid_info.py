from uipc import builtin 

# only export ConstitutionUIDInfo and ImplicitGeometryUIDInfo
__all__ = ['ConstitutionUIDInfo', 'ImplicitGeometryUIDInfo']

class UIDInfo:    
    def check_uid_available(self, uid: int) -> bool:
        uids = self.uids
        uid_map = {u['uid']:u for u in uids}
        if uid in uid_map.keys():
            u = uid_map[uid]
            name = u['name']
            type = u['type']
            # print with red color
            print(f'\033[91mUID Already Exists:\n{uid} |  {name} | {type}\033[0m')
            return False
        return True

    def first_available_uid(self, start_uid: int=0) -> int:
        # sort uids by uid
        uids = self.uids
        uid_set = {u['uid'] for u in uids}
        uid = start_uid
        while uid in uid_set:
            uid += 1
        return uid
    
    def __init__(self, uids):
        self.uids = sorted(uids, key=lambda x: x['uid'])
    
    def __repr__(self):
        CONTENT = '''| UID | Name | Type |
|-----|------|------|'''
        for u in self.uids:
            CONTENT += f'\n| {u["uid"]} | {u["name"]} | {u["type"]} |'
        return CONTENT

class ConstitutionUIDInfo(UIDInfo):
    def __init__(self):
        super().__init__(builtin.ConstitutionUIDCollection.instance().to_json())

class ImplicitGeometryUIDInfo(UIDInfo):
    def __init__(self):
        super().__init__(builtin.ImplicitGeometryUIDCollection.instance().to_json())

if __name__ == '__main__':
    CInfo = ConstitutionUIDInfo()
    print(CInfo)
    print(f'First available UID: {CInfo.first_available_uid(100)}')
    print(f'Check UID 100 available: {CInfo.check_uid_available(100)}')

    IInfo = ImplicitGeometryUIDInfo()
    print(IInfo)
    print(f'First available UID: {IInfo.first_available_uid(100)}')
    print(f'Check UID 100 available: {IInfo.check_uid_available(100)}')
