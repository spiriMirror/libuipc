from uipc import builtin 

# only export ConstitutionUIDInfo and ImplicitGeometryUIDInfo
__all__ = ['ConstitutionUIDInfo', 'ImplicitGeometryUIDInfo']

class UIDInfo:
    def __init__(self, uids):
        self.uids = sorted(uids, key=lambda x: x['uid'])
        self._uid_map = {u['uid']: u for u in self.uids}

    def get_uid_info(self, uid: int):
        """Gets information about a given UID."""
        return self._uid_map.get(uid)
    
    def check_uid_available(self, uid: int) -> bool:
        """Checks if a UID is available (i.e., not already in use)."""
        return uid not in self._uid_map

    def first_available_uid(self, start_uid: int=0) -> int:
        uid = start_uid
        # Using self._uid_map is much faster than creating a set every time.
        while uid in self._uid_map:
            uid += 1
        return uid

    def __repr__(self):
        header = "| UID | Name | Type |\n|-----|------|------|"
        # Use a list comprehension and join for efficient string building.
        rows = [f'| {u["uid"]} | {u["name"]} | {u["type"]} |' for u in self.uids]
        return "\n".join([header] + rows)

class ConstitutionUIDInfo(UIDInfo):
    def __init__(self):
        super().__init__(builtin.ConstitutionUIDCollection.instance().to_json())

class ImplicitGeometryUIDInfo(UIDInfo):
    def __init__(self):
        super().__init__(builtin.ImplicitGeometryUIDCollection.instance().to_json())

