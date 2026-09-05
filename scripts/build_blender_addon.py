"""Build a deterministic, dependency-free Blender extension ZIP.

The external solver environment is installed by the user, independently of
Blender. No native binaries or wheel copies are embedded into this extension.
"""

import argparse
from pathlib import Path
import tomllib
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("output/blender-dist"))
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1] / "integrations/blender/libuipc_blender"
    manifest = tomllib.loads((source / "blender_manifest.toml").read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"{manifest['id']}-{manifest['version']}.zip"
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source.rglob("*")):
            if not path.is_file() or "__pycache__" in path.parts or path.suffix == ".pyc":
                continue
            info = zipfile.ZipInfo(path.relative_to(source).as_posix(), date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, path.read_bytes())
    print(output.resolve())


if __name__ == "__main__":
    main()
