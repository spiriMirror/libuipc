import uipc
from uipc.dev import ConstitutionUIDInfo, ImplicitGeometryUIDInfo
from uipc import unit

if __name__ == '__main__':
    print('Constitutions:')
    print(ConstitutionUIDInfo())
    print('-'*80)
    print('Implicit Geometries:')
    print(ImplicitGeometryUIDInfo())
    print('-'*80)
    print('Units:')
    print(f's={unit.s}')
    print(f'm={unit.m}')
    print(f'mm={unit.mm}')
    print(f'km={unit.km}')
    print(f'Pa={unit.Pa}')
    print(f'kPa={unit.kPa}')
    print(f'MPa={unit.MPa}')
    print(f'GPa={unit.GPa}')