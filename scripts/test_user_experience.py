#!/usr/bin/env python3
"""
User Experience Test Script for LibUIPC
Simulates end-user installation and usage scenarios
"""
import os
import sys
import subprocess
import tempfile
import shutil
from pathlib import Path

class UserExperienceTest:
    def __init__(self, wheel_path=None, use_pypi=True):
        """
        Initialize test with wheel path or PyPI installation
        
        Args:
            wheel_path: Path to local wheel file (optional)
            use_pypi: Use PyPI installation if True, local wheel if False
        """
        self.wheel_path = wheel_path
        self.use_pypi = use_pypi
        self.test_env = None
        
    def create_clean_environment(self):
        """Create a clean virtual environment for testing"""
        print("Creating clean test environment...")
        
        # Create temporary directory for virtual environment
        self.test_env = tempfile.mkdtemp(prefix="libuipc_test_")
        venv_path = os.path.join(self.test_env, "venv")
        
        # Create virtual environment
        result = subprocess.run([
            sys.executable, "-m", "venv", venv_path
        ], capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"Failed to create virtual environment: {result.stderr}")
            return False
            
        # Get Python executable path
        if os.name == 'nt':  # Windows
            self.python_exe = os.path.join(venv_path, "Scripts", "python.exe")
            self.pip_exe = os.path.join(venv_path, "Scripts", "pip.exe")
        else:  # Unix-like
            self.python_exe = os.path.join(venv_path, "bin", "python")
            self.pip_exe = os.path.join(venv_path, "bin", "pip")
            
        print(f"[OK] Created virtual environment at: {venv_path}")
        return True
        
    def install_package(self):
        """Install the package in the clean environment"""
        print("\nInstalling LibUIPC package...")
        
        if self.use_pypi:
            # Install from PyPI
            cmd = [self.pip_exe, "install", "pyuipc"]
            print("Installing from PyPI...")
        else:
            # Install from local wheel
            if not self.wheel_path or not os.path.exists(self.wheel_path):
                print("Error: Wheel path not provided or doesn't exist")
                return False
            cmd = [self.pip_exe, "install", self.wheel_path]
            print(f"Installing from local wheel: {self.wheel_path}")
            
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"[ERROR] Installation failed:")
            print(f"STDOUT: {result.stdout}")
            print(f"STDERR: {result.stderr}")
            return False
        else:
            print("[OK] Installation completed successfully")
            print(f"Output: {result.stdout}")
            return True
    
    def test_basic_import(self):
        """Test basic package import"""
        print("\nTesting basic import...")
        
        test_script = '''
import sys
try:
    import uipc
    print("[OK] Successfully imported uipc")
    
    # Try to access package info
    if hasattr(uipc, '__version__'):
        print(f"[OK] Package version: {uipc.__version__}")
    else:
        print("? No version information found")
        
    print(f"[OK] Package location: {uipc.__file__}")
    sys.exit(0)
except ImportError as e:
    print(f"[ERROR] Failed to import uipc: {e}")
    sys.exit(1)
except Exception as e:
    print(f"[ERROR] Unexpected error: {e}")
    sys.exit(1)
'''
        
        result = subprocess.run([
            self.python_exe, "-c", test_script
        ], capture_output=True, text=True)
        
        print(result.stdout)
        if result.stderr:
            print(f"Warnings/Errors: {result.stderr}")
            
        return result.returncode == 0
    
    def test_stub_files(self):
        """Test that type stub files are accessible"""
        print("\nTesting type stub files...")
        
        test_script = '''
import os
import uipc

# Get package location
package_path = os.path.dirname(uipc.__file__)
print(f"Package path: {package_path}")

# Expected stub files
stub_files = [
    '__init__.pyi', 'backend.pyi', 'builtin.pyi',
    'constitution.pyi', 'core.pyi', 'diff_sim.pyi',
    'geometry.pyi', 'unit.pyi'
]

# Look for pyuipc stub directory
potential_dirs = [
    os.path.join(package_path, '..', 'pyuipc'),
    os.path.join(os.path.dirname(package_path), 'pyuipc'),
]

stub_dir = None
for potential_dir in potential_dirs:
    if os.path.exists(potential_dir):
        stub_dir = potential_dir
        break

if not stub_dir:
    print("[ERROR] pyuipc stub directory not found")
    exit(1)

print(f"[OK] Found stub directory: {stub_dir}")

missing_stubs = []
for stub_file in stub_files:
    stub_path = os.path.join(stub_dir, stub_file)
    if not os.path.exists(stub_path):
        missing_stubs.append(stub_file)

if missing_stubs:
    print(f"[ERROR] Missing stub files: {missing_stubs}")
    exit(1)
else:
    print(f"[OK] All {len(stub_files)} stub files found")
    exit(0)
'''
        
        result = subprocess.run([
            self.python_exe, "-c", test_script
        ], capture_output=True, text=True)
        
        print(result.stdout)
        if result.stderr:
            print(f"Warnings/Errors: {result.stderr}")
            
        return result.returncode == 0
    
    def test_sample_usage(self):
        """Test basic functionality usage"""
        print("\nTesting sample usage...")
        
        test_script = '''
try:
    import uipc
    print("[OK] Package imported successfully")
    
    # Test basic functionality (if available)
    # Add specific tests based on LibUIPC API
    
    # For now, just check if basic attributes/modules are accessible
    attrs_to_check = ['__file__', '__name__']
    for attr in attrs_to_check:
        if hasattr(uipc, attr):
            print(f"[OK] Has attribute: {attr}")
        
    print("[OK] Basic functionality test completed")
    exit(0)
    
except Exception as e:
    print(f"[ERROR] Sample usage test failed: {e}")
    import traceback
    traceback.print_exc()
    exit(1)
'''
        
        result = subprocess.run([
            self.python_exe, "-c", test_script
        ], capture_output=True, text=True)
        
        print(result.stdout)
        if result.stderr:
            print(f"Warnings/Errors: {result.stderr}")
            
        return result.returncode == 0
    
    def test_documentation_access(self):
        """Test if documentation/help is accessible"""
        print("\nTesting documentation access...")
        
        test_script = '''
try:
    import uipc
    
    # Try to get help information
    help_info = help(uipc)
    print("[OK] Help information accessible")
    
    # Check for docstrings
    if uipc.__doc__:
        print(f"[OK] Package docstring: {uipc.__doc__[:100]}...")
    else:
        print("? No package docstring found")
        
    exit(0)
except Exception as e:
    print(f"[ERROR] Documentation access test failed: {e}")
    exit(1)
'''
        
        result = subprocess.run([
            self.python_exe, "-c", test_script
        ], capture_output=True, text=True, timeout=30)
        
        print(result.stdout)
        if result.stderr:
            print(f"Warnings/Errors: {result.stderr}")
            
        return result.returncode == 0
    
    def cleanup(self):
        """Clean up test environment"""
        if self.test_env and os.path.exists(self.test_env):
            try:
                shutil.rmtree(self.test_env)
                print(f"\n[OK] Cleaned up test environment: {self.test_env}")
            except Exception as e:
                print(f"\n? Warning: Failed to clean up {self.test_env}: {e}")
    
    def run_full_test(self):
        """Run the complete user experience test suite"""
        print("=" * 60)
        print("LibUIPC User Experience Test")
        print("=" * 60)
        
        if self.use_pypi:
            print("Testing PyPI installation...")
        else:
            print(f"Testing local wheel: {self.wheel_path}")
        
        tests = [
            ("Environment Setup", self.create_clean_environment),
            ("Package Installation", self.install_package),
            ("Basic Import", self.test_basic_import),
            ("Stub Files", self.test_stub_files),
            ("Sample Usage", self.test_sample_usage),
            ("Documentation Access", self.test_documentation_access),
        ]
        
        results = {}
        
        try:
            for test_name, test_func in tests:
                print(f"\n{'='*20} {test_name} {'='*20}")
                try:
                    success = test_func()
                    results[test_name] = success
                    if not success:
                        print(f"[ERROR] {test_name} failed - stopping tests")
                        break
                except Exception as e:
                    print(f"[ERROR] {test_name} failed with exception: {e}")
                    results[test_name] = False
                    break
        finally:
            self.cleanup()
        
        # Print summary
        print("\n" + "=" * 60)
        print("TEST RESULTS SUMMARY")
        print("=" * 60)
        
        passed = sum(1 for result in results.values() if result)
        total = len(results)
        
        for test_name, success in results.items():
            status = "[PASS]" if success else "[FAIL]"
            print(f"{test_name:25} {status}")
        
        print("-" * 60)
        print(f"Overall: {passed}/{total} tests passed")
        
        if passed == total:
            print("All tests passed! User experience looks good.")
            return True
        else:
            print("Some tests failed. User experience needs improvement.")
            return False

def main():
    """Main function to run user experience tests"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Test LibUIPC user experience")
    parser.add_argument("--wheel", help="Path to local wheel file")
    parser.add_argument("--pypi", action="store_true", default=True, 
                       help="Test PyPI installation (default)")
    parser.add_argument("--local", action="store_true", 
                       help="Test local wheel installation")
    
    args = parser.parse_args()
    
    if args.local and not args.wheel:
        print("Error: --local requires --wheel parameter")
        return 1
    
    use_pypi = not args.local
    tester = UserExperienceTest(wheel_path=args.wheel, use_pypi=use_pypi)
    
    success = tester.run_full_test()
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())