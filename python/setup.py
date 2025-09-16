import os
import sys
import shutil
import subprocess
import platform
from pathlib import Path
from setuptools import setup
from setuptools.dist import Distribution
from setuptools.command.build_py import build_py


class BinaryDistribution(Distribution):
    '''Distribution which always forces a binary package with platform name'''
    def has_ext_modules(foo):
        return True


class BuildPyCommand(build_py):
    """Custom build command that handles dependency copying and RPATH fixes."""

    def run(self):
        # Run the standard build first
        super().run()

        # Perform post-build operations
        self.copy_dependencies()
        self.fix_rpath()

    def copy_dependencies(self):
        """Copy vcpkg dependencies and shared libraries to modules directory."""
        print("Copying dependencies and setting up modules directory...")

        # Determine platform-specific settings
        if platform.system() == "Linux":
            vcpkg_triplet = "x64-linux"
            shared_lib_ext = ".so*"
        elif platform.system() == "Windows":
            vcpkg_triplet = "x64-windows"
            shared_lib_ext = ".dll"
        else:
            print(f"WARNING: Unsupported platform: {platform.system()}")
            return

        # Find build configuration
        build_config = self._find_build_config()
        if not build_config:
            print("WARNING: No build configuration found, skipping dependency copy")
            return

        # Create modules directory structure
        modules_dir = Path(self.build_lib) / "uipc" / "modules" / build_config / "bin"
        modules_dir.mkdir(parents=True, exist_ok=True)
        print(f"Created modules directory: {modules_dir}")

        # Copy vcpkg dependencies
        self._copy_vcpkg_deps(vcpkg_triplet, shared_lib_ext, modules_dir)

        # Copy other shared libraries
        self._copy_shared_libs(build_config, shared_lib_ext, modules_dir)

    def _find_build_config(self):
        """Find the build configuration directory."""
        possible_configs = ["Release", "RelWithDebInfo", "Debug"]

        # Look for build directories relative to python directory
        build_dirs = [
            Path("../build"),
            Path("../../build"),
            Path("./build"),
        ]

        for build_dir in build_dirs:
            if not build_dir.exists():
                continue

            for config in possible_configs:
                config_dir = build_dir / config / "bin"
                if config_dir.exists():
                    print(f"Found build configuration: {config} in {build_dir}")
                    return config

        print("WARNING: No build configuration directory found")
        return None

    def _copy_vcpkg_deps(self, vcpkg_triplet, shared_lib_ext, modules_dir):
        """Copy vcpkg dependencies to modules directory."""
        vcpkg_paths = [
            Path("../build/vcpkg_installed") / vcpkg_triplet / "bin",
            Path("../build/vcpkg_installed") / vcpkg_triplet / "lib",
            Path("../../build/vcpkg_installed") / vcpkg_triplet / "bin",
            Path("../../build/vcpkg_installed") / vcpkg_triplet / "lib",
        ]

        copied_count = 0
        for vcpkg_path in vcpkg_paths:
            if vcpkg_path.exists():
                print(f"Copying vcpkg dependencies from: {vcpkg_path}")
                for lib_file in vcpkg_path.glob(f"*{shared_lib_ext}"):
                    try:
                        shutil.copy2(lib_file, modules_dir)
                        print(f"  Copied {lib_file.name}")
                        copied_count += 1
                    except Exception as e:
                        print(f"  WARNING: Failed to copy {lib_file.name}: {e}")

        if copied_count == 0:
            print("WARNING: No vcpkg dependencies found to copy")

    def _copy_shared_libs(self, build_config, shared_lib_ext, modules_dir):
        """Copy other shared libraries to modules directory."""
        lib_paths = [
            Path("../build") / build_config / "bin",
            Path("../../build") / build_config / "bin",
        ]

        copied_count = 0
        for lib_path in lib_paths:
            if lib_path.exists():
                print(f"Copying shared libraries from: {lib_path}")
                for lib_file in lib_path.glob(f"*{shared_lib_ext}"):
                    try:
                        shutil.copy2(lib_file, modules_dir)
                        print(f"  Copied {lib_file.name}")
                        copied_count += 1
                    except Exception as e:
                        print(f"  WARNING: Failed to copy {lib_file.name}: {e}")

        if copied_count == 0:
            print("WARNING: No shared libraries found to copy")

    def fix_rpath(self):
        """Fix RPATH for shared libraries on Linux."""
        if platform.system() != "Linux":
            print("INFO: RPATH fix only needed on Linux, skipping")
            return

        print("Fixing RPATH for shared libraries...")

        # Find all .so files in the build directory
        fixed_count = 0
        for so_file in Path(self.build_lib).rglob("*.so*"):
            if so_file.is_file() and not so_file.is_symlink():
                try:
                    subprocess.run([
                        "patchelf", "--set-rpath", "$ORIGIN", str(so_file)
                    ], check=True, capture_output=True)
                    print(f"  Fixed RPATH for {so_file.name}")
                    fixed_count += 1
                except subprocess.CalledProcessError as e:
                    print(f"  WARNING: Failed to fix RPATH for {so_file.name}: {e}")
                except FileNotFoundError:
                    print("  WARNING: patchelf not found, skipping RPATH fix")
                    print("     Install patchelf: sudo apt install patchelf")
                    break

        if fixed_count > 0:
            print(f"Fixed RPATH for {fixed_count} libraries")
        else:
            print("INFO: No libraries needed RPATH fixing")


setup(
    distclass=BinaryDistribution,
    cmdclass={
        'build_py': BuildPyCommand,
    },
)