'''CLI entry point for uid_info tool'''
from uipc.dev.uid_info import ConstitutionUIDInfo, ImplicitGeometryUIDInfo
import argparse as ap

def main():
    parser = ap.ArgumentParser(description='UID info tool for displaying UID information')
    args = parser.parse_args()

    print('Constitution UID Info:')
    CInfo = ConstitutionUIDInfo()
    print(CInfo)

    print('Implicit Geometry UID Info:')
    IInfo = ImplicitGeometryUIDInfo()
    print(IInfo)

if __name__ == '__main__':
    main()

