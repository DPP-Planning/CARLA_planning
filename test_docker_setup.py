#!/usr/bin/env python3
"""
Docker Setup Verification Script
Tests if the CARLA Docker setup is working correctly
"""

import sys
import os
import time

def print_status(message, status="INFO"):
    colors = {
        "INFO": "\033[94m",
        "OK": "\033[92m",
        "WARN": "\033[93m",
        "ERROR": "\033[91m",
        "END": "\033[0m"
    }
    print(f"{colors.get(status, '')}{status}: {message}{colors['END']}")

def test_imports():
    """Test if required packages can be imported"""
    print_status("Testing Python package imports...", "INFO")
    
    packages = {
        'carla': 'CARLA Python API',
        'numpy': 'NumPy',
        'shapely': 'Shapely',
        'networkx': 'NetworkX'
    }
    
    failed = []
    for package, name in packages.items():
        try:
            __import__(package)
            print_status(f"  ✓ {name}", "OK")
        except ImportError as e:
            print_status(f"  ✗ {name}: {e}", "ERROR")
            failed.append(package)
    
    if failed:
        print_status(f"Failed to import: {', '.join(failed)}", "ERROR")
        return False
    
    print_status("All required packages imported successfully", "OK")
    return True

def test_carla_connection():
    """Test connection to CARLA server"""
    print_status("Testing CARLA server connection...", "INFO")
    
    try:
        import carla
        
        host = os.getenv('CARLA_HOST', 'localhost')
        port = int(os.getenv('CARLA_PORT', '4000'))
        
        print_status(f"  Attempting to connect to {host}:{port}", "INFO")
        
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        
        # Try to get world
        world = client.get_world()
        print_status(f"  ✓ Connected to CARLA server", "OK")
        
        # Get some basic info
        carla_map = world.get_map()
        map_name = carla_map.name
        print_status(f"  ✓ Current map: {map_name}", "OK")
        
        # Get number of actors
        actors = world.get_actors()
        print_status(f"  ✓ Number of actors in world: {len(actors)}", "OK")
        
        return True
        
    except Exception as e:
        print_status(f"Failed to connect to CARLA: {e}", "ERROR")
        print_status("Make sure CARLA server is running and accessible", "WARN")
        return False

def test_agents_modules():
    """Test if agents.navigation modules are available"""
    print_status("Testing agents.navigation modules...", "INFO")
    
    modules = [
        'agents.navigation.global_route_planner',
        'agents.navigation.basic_agent',
        'agents.navigation.local_planner',
        'agents.tools.misc'
    ]
    
    failed = []
    for module in modules:
        try:
            __import__(module)
            print_status(f"  ✓ {module}", "OK")
        except ImportError as e:
            print_status(f"  ✗ {module}: {e}", "ERROR")
            failed.append(module)
    
    if failed:
        print_status(f"Failed to import: {', '.join(failed)}", "ERROR")
        return False
    
    print_status("All agent modules available", "OK")
    return True

def test_file_structure():
    """Test if expected files and directories exist"""
    print_status("Testing file structure...", "INFO")
    
    expected_paths = [
        'grp planning',
        'grp planning/simple-vehicle.py',
        'agents',
        'agents/navigation',
    ]
    
    missing = []
    for path in expected_paths:
        if os.path.exists(path):
            print_status(f"  ✓ {path}", "OK")
        else:
            print_status(f"  ✗ {path} not found", "WARN")
            missing.append(path)
    
    if missing:
        print_status(f"Missing paths: {', '.join(missing)}", "WARN")
        print_status("This might be okay depending on your setup", "INFO")
    else:
        print_status("All expected files/directories present", "OK")
    
    return True

def main():
    print("\n" + "="*60)
    print("CARLA Planning Docker Setup Verification")
    print("="*60 + "\n")
    
    tests = [
        ("Import Test", test_imports),
        ("Agents Modules Test", test_agents_modules),
        ("File Structure Test", test_file_structure),
        ("CARLA Connection Test", test_carla_connection),
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        try:
            results[test_name] = test_func()
        except Exception as e:
            print_status(f"Test crashed: {e}", "ERROR")
            results[test_name] = False
        time.sleep(0.5)
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    passed = sum(1 for r in results.values() if r)
    total = len(results)
    
    for test_name, result in results.items():
        status = "PASS" if result else "FAIL"
        print_status(f"{test_name}: {status}", "OK" if result else "ERROR")
    
    print(f"\nResults: {passed}/{total} tests passed")
    
    if passed == total:
        print_status("\n✓ All tests passed! Docker setup is working correctly.", "OK")
        return 0
    elif passed >= total - 1 and not results.get("CARLA Connection Test", False):
        print_status("\n⚠ Almost there! CARLA server connection failed.", "WARN")
        print_status("Make sure CARLA server is running:", "INFO")
        print_status("  docker-compose up carla-server", "INFO")
        return 1
    else:
        print_status("\n✗ Some tests failed. Check the errors above.", "ERROR")
        return 1

if __name__ == "__main__":
    sys.exit(main())
