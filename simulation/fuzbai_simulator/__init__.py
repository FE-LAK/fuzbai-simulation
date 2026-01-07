# Top-level package file

from pathlib import Path

import os

# Handle DLL search path on Windows.
# Maturin already handles Linux SOs.
is_windows = os.name == "nt"
if is_windows:
    dll_dir_handle = os.add_dll_directory(str(Path(__file__).parent))

from .fuzbai_simulator import *

if is_windows:
    dll_dir_handle.close()

del Path
del os
