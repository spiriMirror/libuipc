import json
import argparse
import os

base_vcpkg_json = {
    "name": "libuipc",
    "version": "0.0.1",
    "description": "A Modern C++20 Library of Unified Incremental Potential Contact.",
    "builtin-baseline": "943c5ef1c8f6b5e6ced092b242c8299caae2ff01",
    "dependencies": [
        {
            "name": "eigen3",
            "version>=": "3.4.0"
        },
        {
            "name": "catch2",
            "version>=": "3.5.3"
        },
        {
            "name": "libigl",
            "version>=": "2.5.0"
        },
        {
            "name": "spdlog",
            "version>=": "1.12.0"
        },
        {
            "name": "fmt",
            "version>=": "10.2.1"
        },
        {
            "name": "cppitertools",
            "version>=": "2.1#3"
        },
        {
            "name": "dylib",
            "version>=": "2.2.1"
        },
        {
            "name": "nlohmann-json",
            "version>=": "3.11.2"
        },
        {
            "name":"magic-enum",
            "version>=": "0.9.3"
        },
        {
            "name":"boost-core",
            "version>=":"1.84.0"
        },
        {
            "name":"tinygltf",
            "version>=":"2.8.22"
        },
        {
            "name":"tbb",
            "version>=":"2021.11.0"
        }
    ]
}

deps = base_vcpkg_json["dependencies"]

def is_enabled(arg):
    ARG = str(arg).upper()
    if ARG == "ON" or ARG == "1":
        return True
    else:
        return False

def gen_vcpkg_json(args):
    if is_enabled(args.build_gui):
        deps.append({
            "name": "imgui",
            "version>=": "1.90.7"
        })
        deps.append({
            "name": "glfw3",
            "version>=": "3.3.8#2"
        })
        deps.append({
            "name": "opengl",
            "version>=": "2022-12-04#3"
        })
        deps.append({
            "name": "freeglut",
            "version>=": "3.4.0"
        })
        deps.append({
            "name": "bgfx",
            "version>=": "1.127.8725-469"
        })
    
    # NOTE: now we don't use vcpkg to manage pybind11
    # we just use the pip to find its own version
    
    # if is_enabled(args.build_pybind):
    #     deps.append({
    #         "name":"pybind11",
    #         "version>=":"2.12.0"
    #     })

def print_deps():
    str_names = []
    for dep in deps:
        s = "    * " + dep["name"] + " [" + dep["version>="] + "]"
        str_names.append(s)
    str_names = "\n".join(str_names)
    print(f"[libuipc] Writing vcpkg.json with dependencies:\n{str_names}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate vcpkg.json for libuipc.")
    parser.add_argument("output_dir", type=str, help="Output file path.")
    parser.add_argument("--build_gui", type=str, default=False, help="Build GUI dependencies.")
    parser.add_argument("--dev_mode", type=str, default=False, help="Enable development mode.")
    
    args = parser.parse_args()
    print(f"[libuipc] Generating vcpkg.json with args:")
    for K,V in vars(args).items():
        print(f"    * {K}: {V}")
    
    json_path = f'{args.output_dir}/vcpkg.json'
    
    gen_vcpkg_json(args)
    
    is_div_mode = is_enabled(args.dev_mode)
    
    is_new = not os.path.exists(json_path)
    # if json_path exists, compare the content
    changed = False
    
    if not is_new:
        with open(json_path, "r") as f:
            old_json = json.load(f)
            changed = str(old_json) != str(base_vcpkg_json)
    
    with open(json_path, "w") as f:
        json.dump(base_vcpkg_json, f, indent=4)
    
    if is_new:
        print(f"[libuipc] Generated vcpkg.json at:\n    {json_path}")
        print_deps()
        exit(1)
    
    if changed:
        print(f"[libuipc] vcpkg.json content has changed, overwriting:\n    {json_path}")
        print_deps()
        exit(1)
        
    if is_div_mode:
        print(f"[libuipc] vcpkg.json content is unchanged, skipping:\n    {json_path}")
        print_deps()
        exit(0)
    
    print('[libuipc] User mode always try to install dependencies. '
          'If you want to skip, please define `-DUIPC_DEV_MODE=ON` when configuring CMake.')
    print_deps()
    exit(1)
    
    