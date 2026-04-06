import os
import sys

if sys.platform == "win32":
    # On Windows (Python 3.8+), DLL dependencies of .pyd extension modules are
    # only searched in the .pyd's own directory and system directories — NOT parent
    # directories. Since .pyd files live in subdirectories (e.g. xmera/simulation/)
    # but their DLL dependencies (xmera_core.dll, cspice.dll, etc.) are installed
    # to the package root (xmera/), we must register the package directory explicitly.
    _pkg_dir = os.path.dirname(os.path.abspath(__file__))
    os.add_dll_directory(_pkg_dir)
