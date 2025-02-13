from project_dir import project_dir
import argparse as ap
import subprocess as sp

if __name__ == '__main__':
    proj_dir = project_dir()
    parser = ap.ArgumentParser(description='Build the documents')
    doc_default_dir = proj_dir.parent / 'libuipc-doc' / 'docs'
    parser.add_argument('-o', '--output', help='the output dir', default=f'{doc_default_dir}')
    args = parser.parse_args()
    doc_dir = args.output
    print(f'output_dir={doc_default_dir}')
    config_file = proj_dir / 'mkdocs-with-api.yaml'
    print(f'config_file={config_file}')
    Value = sp.call(['mkdocs', 'build', '-f', config_file, '-d', doc_dir], cwd=proj_dir)
    if Value == 0:
        print('Success')
    else:
        print('Failure')