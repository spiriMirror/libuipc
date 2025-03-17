import uipc
from uipc import builtin
from uipc import unit

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
print('-'*80)
print('units:')

print(f's={unit.s}')
print(f'm={unit.m}')
print(f'mm={unit.mm}')
print(f'km={unit.km}')
print(f'Pa={unit.Pa}')
print(f'kPa={unit.kPa}')
print(f'MPa={unit.MPa}')
print(f'GPa={unit.GPa}')





