#!/usr/bin/env python3
"""
Utility script to compile a MuJoCo XML model to MJB format.
This is needed on macOS where the compile binary is not included in the DMG.
"""

import sys
import os

def compile_model_via_mujoco():
    """Try to compile using the mujoco Python package (mujoco >= 3.0)"""
    try:
        import pathlib
        import mujoco
        
        xml_path = pathlib.Path("models/miza.xml")
        mjb_path = pathlib.Path("simulation/src/miza.mjb")
        
        if not xml_path.exists():
            print(f"Error: {xml_path} not found", file=sys.stderr)
            return False
        
        print(f"Compiling {xml_path} to {mjb_path} using mujoco Python package...")
        
        # Load and compile the model
        model = mujoco.MjModel.from_xml_file(str(xml_path))
        
        # Save as MJB
        model.save(str(mjb_path))
        
        print(f"Successfully compiled model to {mjb_path}")
        return True
    except ImportError:
        print("mujoco Python package not found", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Error compiling model: {e}", file=sys.stderr)
        return False

def compile_model_via_subprocess():
    """Try to compile using an external MuJoCo compile binary"""
    import subprocess
    import shutil
    
    xml_path = "models/miza.xml"
    mjb_path = "simulation/src/miza.mjb"
    
    # Try to find compile binary
    compile_bin = shutil.which("mjcompile") or "./mujoco-3.3.7/bin/compile"
    
    if not os.path.exists(compile_bin):
        print(f"MuJoCo compile binary not found at {compile_bin}", file=sys.stderr)
        return False
    
    try:
        print(f"Compiling {xml_path} to {mjb_path} using {compile_bin}...")
        subprocess.run([compile_bin, xml_path, mjb_path], check=True)
        print(f"Successfully compiled model to {mjb_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error running compile: {e}", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return False

def main():
    print("Attempting to compile MuJoCo model...")
    
    # Try Python package first (easiest)
    if compile_model_via_mujoco():
        return 0

    print("\nError: Could not compile MuJoCo model.", file=sys.stderr)
    print("Please ensure one of the following is available:", file=sys.stderr)
    print("  1. Python mujoco package (pip install mujoco)", file=sys.stderr)
    print("  2. MuJoCo compile binary in PATH or at ./mujoco-3.3.7/bin/compile", file=sys.stderr)
    return 1

if __name__ == "__main__":
    sys.exit(main())
