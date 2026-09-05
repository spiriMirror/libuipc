# SPDX-License-Identifier: Apache-2.0
"""Install the built ZIP through Blender's real extension operator.

Use isolated BLENDER_USER_CONFIG/BLENDER_USER_EXTENSIONS for validation. Pass
--persist only when intentionally installing in the user's Blender profile.
"""

import argparse
import importlib
from pathlib import Path
import runpy
import sys

import bpy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zip", type=Path, required=True)
    parser.add_argument("--python", required=True)
    parser.add_argument("--persist", action="store_true")
    parser.add_argument("--test", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    repo = next(r for r in bpy.context.preferences.extensions.repos if r.module == "user_default")
    assert bpy.ops.extensions.package_install_files(filepath=str(args.zip.resolve()),
        repo=repo.module, enable_on_install=True) == {"FINISHED"}
    module = "bl_ext." + repo.module + ".libuipc_blender"
    addon = importlib.import_module(module)
    bpy.context.preferences.addons[module].preferences.python_executable = args.python
    assert "uipc" not in sys.modules, "Native solver must not load into Blender's Python"
    assert hasattr(bpy.types.Scene, "uipc_settings")
    print("BLENDER_EXTENSION_INSTALLED", addon.__file__)
    if args.persist:
        assert bpy.ops.wm.save_userpref() == {"FINISHED"}
    if args.test:
        sys.argv = [str(args.test), "--", "--module", module, "--python", args.python,
                    "--output", str(args.output.resolve())]
        if args.quick:
            sys.argv.append("--quick")
        runpy.run_path(str(args.test.resolve()), run_name="__main__")


if __name__ == "__main__":
    main()
