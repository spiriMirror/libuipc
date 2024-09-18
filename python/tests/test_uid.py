from pyuipc_loader import pyuipc as uipc
import pytest 

@pytest.mark.basic
def test_uid_collection():
    constitutions = uipc.builtin.ConstitutionUIDCollection.instance().to_json()
    implicit_geomeries = uipc.builtin.ImplicitGeometryUIDCollection.instance().to_json()

    def print_sorted(uids):
        uids = sorted(uids, key=lambda x: x['uid'])
        for u in uids:
            uid = u['uid']
            name = u['name']
            print(f'uid: {uid}, name: {name}')

    print('constitutions:')
    print_sorted(constitutions)
    print('-'*80)
    print('implicit_geomeries:')
    print_sorted(implicit_geomeries)
