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
        model = mujoco.MjModel.from_xml_path(str(xml_path))

        # Save as MJB using mj_saveModel
        mujoco.mj_saveModel(model, str(mjb_path))
        return True
    except ImportError:
        print("mujoco Python package not found", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Error compiling model: {e}", file=sys.stderr)
        return False

def main():   
    # Try Python package first (easiest)
    if compile_model_via_mujoco():
        return 0

    print("\nError: Could not compile MuJoCo model.", file=sys.stderr)
    print("Please ensure one of the following is available:", file=sys.stderr)
    print("  1. Python mujoco package (pip install mujoco~=3.3.7)", file=sys.stderr)
    return 1

if __name__ == "__main__":
    sys.exit(main())
