'''CLI entry point for mesh_doctor tool'''
import sys
import os
import argparse as ap

from uipc.dev.mesh_doctor import MeshDoctor
from uipc.constitution import AffineBodyConstitution
from uipc.geometry import SimplicialComplexIO
import uipc


def main():
    '''CLI entry point for mesh doctor utility'''
    parser = ap.ArgumentParser(description='Mesh doctor utility for checking mesh validity')
    parser.add_argument('mesh', type=str, help='Path to the mesh file to check')
    parser.add_argument('--workspace', type=str, default=None, 
                        help='Workspace directory for output files (default: current working directory)')
    parser.add_argument('--abd', action='store_true', 
                        help='Use AffineBodyConstitution for checking')
    parser.add_argument('--gui', action='store_true', default=True,
                        help='Enable GUI visualization (default: True)')
    
    args = parser.parse_args()
    
    # Set workspace to current directory if not provided
    workspace = args.workspace if args.workspace else os.getcwd()
    print(f'Workspace: {workspace}')
    
    doctor = MeshDoctor(workspace, with_gui=args.gui)

    if not os.path.exists(args.mesh):
        uipc.Logger.error(f'Mesh file {args.mesh} does not exist')
        sys.exit(1)
    
    io = SimplicialComplexIO()
    mesh = io.read(args.mesh)
    
    if args.abd:
        constitution = AffineBodyConstitution()
    else:
        raise ValueError('Please specify a constitution type (e.g., --abd for AffineBodyConstitution)')
    
    is_valid = doctor.check_mesh(constitution, mesh)
    
    if is_valid:
        uipc.Logger.info('Mesh check passed!')
        sys.exit(0)
    else:
        uipc.Logger.error('Mesh check failed! See visualization for details.')
        sys.exit(1)

if __name__ == '__main__':
    main()

