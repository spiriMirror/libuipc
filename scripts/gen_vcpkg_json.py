import json
import argparse
import os

VCPKG_TAG = '2025.7.25'
VCPKG_BASE_LINE = 'dd3097e305afa53f7b4312371f62058d2e665320'

SPIRI_VCPKG_BASE_LINE = 'c34f15fe5867f61cb83309fd053da6b47eca0af9'

# vcpkg.json
base_vcpkg_json = {
    'name': 'libuipc',
    'version': '0.0.1',
    'description': 'A Modern C++20 Library of Unified Incremental Potential Contact.',
    'dependencies': [
        {
            'name': 'eigen3',
            'version>=': '3.4.0'
        },
        {
            'name': 'catch2',
            'version>=': '3.5.3'
        },
        {
            'name': 'libigl',
            'version>=': '2.5.0'
        },
        {
            'name': 'spdlog',
            'version>=': '1.12.0'
        },
        {
            'name': 'fmt',
            'version>=': '10.2.1'
        },
        {
            'name': 'cppitertools',
            'version>=': '2.1#3'
        },
        {
            'name': 'dylib',
            'version>=': '2.2.1'
        },
        {
            'name': 'nlohmann-json',
            'version>=': '3.11.2'
        },
        {
            'name':'magic-enum',
            'version>=': '0.9.3'
        },
        {
            'name':'tinygltf',
            'version>=':'2.8.22'
        },
        {
            'name':'tbb',
            'version>=':'2022.0.0'
        },
        {
            'name':'urdfdom',
            'version>=':'3.1.1'
        },
        {
            'name':'cpptrace',
            'version>=': '0.8.3'
        },
        {
            'name': 'muda',
            'version>=': '2025.10.3'
        }
    ],
    
    'overrides':[
        # fix fmt version
        {
            'name': 'fmt',
            'version': '10.2.1',
        },
        # fix spdlog version
        {
            'name': 'spdlog',
            'version': '1.12.0',
        }
    ]
}

# vcpkg-configuration.json
base_vcpkg_configuration = {
    'default-registry': {
        'kind': 'git',
        'baseline': VCPKG_BASE_LINE,
        'repository': 'https://github.com/microsoft/vcpkg'
    },
    'registries': [
        {
            'kind': 'git',
            'repository': 'https://github.com/spiriMirror/vcpkg',
            'reference': 'master',
            'baseline': SPIRI_VCPKG_BASE_LINE,
            'packages': [
                'muda'
            ]
        }
    ]
}

def is_enabled(arg):
    ARG = str(arg).upper()
    if ARG == 'ON' or ARG == '1':
        return True
    else:
        return False

def gen_vcpkg_json(args):
    deps = base_vcpkg_json['dependencies']
    
    if is_enabled(args.build_gui):
        deps.append({
            'name': 'imgui',
            'version>=': '1.90.7'
        })
        deps.append({
            'name': 'glfw3',
            'version>=': '3.3.8#2'
        })
        deps.append({
            'name': 'opengl',
            'version>=': '2022-12-04#3'
        })
        deps.append({
            'name': 'freeglut',
            'version>=': '3.4.0'
        })
        deps.append({
            'name': 'bgfx',
            'version>=': '1.127.8725-469'
        })
    if is_enabled(args.with_usd_support):
        deps.append({
            'name': 'usd',
            'version>=': '25.5.1'
        })
    if is_enabled(args.with_vdb_support):
        deps.append({
            'name': 'openvdb',
            'version>=': '12.0.1',
            'features': ['nanovdb']
        })

def print_deps():
    str_names = []
    deps = base_vcpkg_json['dependencies']
    for dep in deps:
        s = '    * ' + dep['name'] + ' [' + dep['version>='] + ']'
        str_names.append(s)
    str_names = '\n'.join(str_names)
    print(f'[libuipc] Writing vcpkg.json with dependencies:\n{str_names}')

def print_basic_info(args):
    print('[libuipc] Generating vcpkg manifest with args:')
    for K,V in vars(args).items():
        print(f'    * {K}: {V}')
    print('[libuipc] Vcpkg Tag:', VCPKG_TAG)

def write_vcpkg_configuration(args):
    config_path = f'{args.output_dir}/vcpkg-configuration.json'
    with open(config_path, 'w') as f:
        json.dump(base_vcpkg_configuration, f, indent=4)
    print(f'[libuipc] Generated vcpkg-configuration.json at:\n    {config_path}')

def write_vcpkg_json(args):
    json_path = f'{args.output_dir}/vcpkg.json'
    
    gen_vcpkg_json(args)
    
    is_dev_mode = is_enabled(args.dev_mode)
    
    is_new = not os.path.exists(json_path)
    # if json_path exists, compare the content
    changed = False
    
    if not is_new:
        with open(json_path, 'r') as f:
            old_json = json.load(f)
            changed = str(old_json) != str(base_vcpkg_json)
    
    with open(json_path, 'w') as f:
        json.dump(base_vcpkg_json, f, indent=4)
        
    if is_new:
        print(f'[libuipc] Generated vcpkg.json at:\n    {json_path}')
        print_deps()
        return 1
    
    if changed:
        print(f'[libuipc] vcpkg.json content has changed, overwriting:\n    {json_path}')
        print_deps()
        return 1
        
    if is_dev_mode:
        print(f'[libuipc] vcpkg.json content is unchanged, skipping:\n    {json_path}')
        print_deps()
        return 0
    
    print('[libuipc] User mode always try to install dependencies. '
          'If you want to skip, please define `-DUIPC_DEV_MODE=ON` when configuring CMake.')
    print_deps()
    return 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate vcpkg.json for libuipc.')
    parser.add_argument('output_dir', type=str, help='Output file path.')
    parser.add_argument('--build_gui', type=str, default=False, help='Build GUI dependencies.')
    parser.add_argument('--dev_mode', type=str, default=False, help='Enable development mode.')
    parser.add_argument('--with_usd_support', type=str, default=False, help='Enable USD support.')
    parser.add_argument('--with_vdb_support', type=str, default=False, help='Enable VDB support.')
    args = parser.parse_args()

    print_basic_info(args)
    
    write_vcpkg_configuration(args)
    
    ret_code = write_vcpkg_json(args)
    
    exit(ret_code)