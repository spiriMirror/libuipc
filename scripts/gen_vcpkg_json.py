import json
import argparse

vcpkg_json = {
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
            "version>=": "10.1.1"
        },
        {
            "name": "cppitertools",
            "version>=": "2.1#3"
        },
        {
            "name": "bgfx",
            "version>=": "1.127.8725-469"
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
        }
    ]
}
    
def gen_vcpkg_json(args):
    deps = vcpkg_json["dependencies"]
    if args.build_gui:
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


if __name__ == "__main__":
    print("-----------------------------------------------------------------------------------")
    parser = argparse.ArgumentParser(description="Generate vcpkg.json for libuipc.")
    parser.add_argument("output_dir", type=str, help="Output file path.")
    parser.add_argument("--build_gui", type=bool, default=False, help="Build GUI dependencies.")
    
    args = parser.parse_args()
    print(f"[libuipc] Generating vcpkg.json with args:")
    for K,V in vars(args).items():
        print(f"    * {K}: {V}")
    
    json_path = f'{args.output_dir}/vcpkg.json'
    
    gen_vcpkg_json(args)
    
    deps = vcpkg_json["dependencies"]
    str_names = []
    for dep in deps:
        s = "    * " + dep["name"] + " [" + dep["version>="] + "]"
        str_names.append(s)
    str_names = "\n".join(str_names)
    print(f"[libuipc] Writing vcpkg.json with dependencies:\n{str_names}")
    
    with open(json_path, "w") as f:
        json.dump(vcpkg_json, f, indent=4)
    
    print(f"[libuipc] Generated vcpkg.json at:\n    {json_path}")
    print("-----------------------------------------------------------------------------------")