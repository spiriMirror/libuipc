from pyuipc_loader import pyuipc
from pyuipc import builtin

constitutions = builtin.ConstitutionUIDCollection.instance().to_json()
implicit_geomeries = builtin.ImplicitGeometryUIDCollection.instance().to_json()

def print_sorted(uids):
    uids = sorted(uids, key=lambda x: x['uid'])
    for u in uids:
        uid = u['uid']
        name = u['name']
        type = u['type']
        print(f'uid: {uid}, name: {name}, type: {type}')

print('constitutions:')
print_sorted(constitutions)
print('-'*80)
print('implicit_geomeries:')
print_sorted(implicit_geomeries)