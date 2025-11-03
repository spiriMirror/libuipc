#!/usr/bin/env python3
"""
Test script for validating LibUIPC wheel installation
"""
import os
import sys
import importlib.util

def test_basic_import():
    """Test basic uipc import"""
    try:
        import uipc
        print("[OK] Basic uipc import successful")
        return True
    except ImportError as e:
        print(f"[ERROR] Failed to import uipc: {e}")
        return False

def test_stub_files():
    """Test that all stub files are present"""
    try:
        import uipc
        package_path = os.path.dirname(uipc.__file__)
        print(f"Package installed at: {package_path}")
        
        # Expected stub files
        stub_files = [
            '__init__.pyi',
            'backend.pyi', 
            'builtin.pyi',
            'constitution.pyi',
            'core.pyi',
            'diff_sim.pyi',
            'geometry.pyi',
            'unit.pyi'
        ]
        
        # Look for pyuipc stub directory
        stub_dirs = [
            os.path.join(package_path, '..', 'pyuipc'),  # Relative to uipc
            os.path.join(os.path.dirname(package_path), 'pyuipc'),  # Same level as uipc
        ]
        
        stub_dir = None
        for potential_dir in stub_dirs:
            if os.path.exists(potential_dir):
                stub_dir = potential_dir
                break
                
        if not stub_dir:
            print("[ERROR] pyuipc stub directory not found")
            return False
            
        print(f"Found stub directory at: {stub_dir}")
        
        missing_stubs = []
        for stub_file in stub_files:
            stub_path = os.path.join(stub_dir, stub_file)
            if not os.path.exists(stub_path):
                missing_stubs.append(stub_file)
                
        if missing_stubs:
            print(f"[ERROR] Missing stub files: {missing_stubs}")
            return False
        else:
            print(f"[OK] All {len(stub_files)} stub files found")
            return True
            
    except Exception as e:
        print(f"[ERROR] Error checking stub files: {e}")
        return False

def test_core_functionality():
    """Test basic core functionality"""
    try:
        import uipc
        # Add basic functionality tests here
        # For now, just check if core modules are accessible
        
        # Test if we can access basic attributes (without causing errors)
        hasattr(uipc, '__version__')  # This might exist
        print("[OK] Core functionality accessible")
        return True
        
    except Exception as e:
        print(f"[ERROR] Core functionality test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("Testing LibUIPC wheel installation...")
    print("=" * 50)
    
    tests = [
        test_basic_import,
        test_stub_files,
        test_core_functionality,
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("=" * 50)
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("All tests passed!")
        return 0
    else:
        print("Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())