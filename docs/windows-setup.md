# Windows Development Environment Setup

This guide walks you through setting up a Windows development environment for xmera.

## Prerequisites

- Windows 10/11 (x64 or ARM64)
- Administrator access for initial setup

## 1. Install Required Tools

### Visual Studio 2022 or later (full Community/Professional/Enterprise edition)

Download from: https://visualstudio.microsoft.com/

> **Important**: Install the full **Community, Professional, or Enterprise** edition.
> The **Build Tools** edition does not include all required toolchains and will cause
> vcpkg to fail.

Install with the following workload:
- **Desktop development with C++**

Ensure these individual components are selected:
- MSVC v143 (or latest) C++ build tools
- C++ CMake tools for Windows
- Windows SDK (latest)
- C++ ATL for latest build tools

### CMake (3.27+)

Download and install from: https://cmake.org/download/

During installation, select **"Add CMake to the system PATH for all users"**.

### Ninja Build System

```powershell
winget install Ninja-build.Ninja
```

Or download from https://github.com/ninja-build/ninja/releases, extract to `C:\tools\ninja`, and add to PATH.

### SWIG (4.0+)

1. Download swigwin from https://www.swig.org/download.html
2. Extract to `C:\tools\swig`
3. Add `C:\tools\swig` to your system PATH

### Python (3.10+)

Download from https://www.python.org/downloads/

**Important**: Choose the architecture matching your build target:
- For x64 builds: Install **Windows installer (64-bit)**
- For ARM64 builds: Install **Windows installer (ARM64)**

During installation:
- Check **"Add Python to PATH"**
- Check **"Install for all users"** (recommended)

### Git

```powershell
winget install Git.Git
```

### vcpkg

Install vcpkg to a short path to avoid Windows path length issues:

```powershell
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

## 2. Configure Environment Variables

Open an elevated PowerShell (`Win` → type `powershell` → `Ctrl+Shift+Enter`) and run:

```powershell
# Required: point to your vcpkg installation
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\vcpkg", "Machine")

# Required on ARM64 Windows: ensures vcpkg finds the correct full VS installation
# (not the Build Tools instance). Adjust the path for your VS version/edition.
[Environment]::SetEnvironmentVariable("VCPKG_VISUAL_STUDIO_PATH", "C:\Program Files\Microsoft Visual Studio\2022\Community", "Machine")
```

Verify PATH also includes:
- `C:\tools\ninja` (if manually installed)
- `C:\tools\swig`
- Python installation directory

**Restart your terminal** after setting environment variables.

## 3. Configure VS Code Terminal

To get the VS Developer environment automatically in every VS Code terminal:

Open `Ctrl+Shift+P` → **"Open User Settings (JSON)"** and add:

```json
"terminal.integrated.profiles.windows": {
    "PowerShell with VS": {
        "source": "PowerShell",
        "args": ["-NoExit", "-Command",
            "Import-Module 'C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\Common7\\Tools\\Microsoft.VisualStudio.DevShell.dll'; Enter-VsDevShell -VsInstallPath 'C:\\Program Files\\Microsoft Visual Studio\\2022\\Community' -SkipAutomaticLocation"
        ]
    }
},
"terminal.integrated.defaultProfile.windows": "PowerShell with VS"
```

Adjust the path if your VS is installed under a different version/edition.

Verify it works by opening a new terminal and running:
```powershell
$env:INCLUDE   # Must print MSVC include paths, not be empty
```

## 4. Directory Structure

Use short paths to avoid Windows 260-character path limit:

```
C:\dev\                    # Development root
└── xmera\                 # This repository

C:\vcpkg\                  # vcpkg package manager
C:\tools\                  # Additional tools (ninja, swig)
C:\build\                  # Optional: build outputs outside the repo
└── xmera\
```

### Clone the repository

```powershell
mkdir C:\dev
cd C:\dev
git clone <your-xmera-repo-url> xmera
```

## 5. Build xmera

### Using the PowerShell build script (Recommended)

The build script handles VS environment initialization, vcpkg, and CMake automatically.

> **Important**: Always build with `-Config Release` for Python use. Python itself is
> a Release binary, and Windows forbids mixing Debug and Release C runtimes in the same
> process. A Debug build will crash with an access violation when Python tries to load
> any `.pyd` extension module.

```powershell
cd C:\dev\xmera
.\build.ps1 -Config Release
```

With options:

```powershell
# Build to a shorter path (avoids path length warnings)
.\build.ps1 -Config Release -BuildDir C:\build\xmera

# Clean build
.\build.ps1 -Config Release -Clean

# Combine options
.\build.ps1 -Config Release -Clean -BuildDir C:\build\xmera
```

### Using CMake directly

Must be run from a terminal where `$env:INCLUDE` is populated (Developer PowerShell):

```powershell
cd C:\dev\xmera\src
cmake --preset windows-ninja
cmake --build ../build --config Release --parallel
```

## 6. Install and run

After a successful build, `cmake --install` is **required** before Python can import
xmera. The install step:
- Copies `.pyd` extension modules and the `xmera_core.dll` to `dist/`
- Copies all vcpkg runtime DLLs (cspice, opencv, zmq, ...) to `dist/lib/`
- Installs a Windows-specific `xmera/__init__.py` that registers `dist/lib/` as a
  DLL search directory so Python can find those DLLs when loading `.pyd` files
- Installs a `cSysModel.py` compatibility shim required by SWIG-generated modules

```powershell
cd C:\dev\xmera

# Required: install build outputs to dist/ (must match the -Config used to build)
cmake --install build --config Release --prefix dist

# Create and activate virtual environment (first time only)
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install xmera as a Python package (development mode, first time only)
pip install -e .

# Run an example
python .\examples\scenarioBasicOrbit.py
```

## 7. PowerShell Execution Policy

If you see "cannot be loaded because running scripts is disabled":

```powershell
# Allow scripts for current user (run once)
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

## Quick Reference

### build.ps1 Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `-Config` | Debug or Release | Debug (use Release for Python) |
| `-Arch` | x64 or arm64 | Auto-detect from Python |
| `-Clean` | Remove build directory first | False |
| `-BuildDir` | Custom build output path | `<repo>\build` |
| `-ConfigureOnly` | Only configure, don't build | False |

### Full workflow from scratch

```powershell
# 1. Build
.\build.ps1 -Config Release -Clean -BuildDir C:\build\xmera

# 2. Install
cmake --install C:\build\xmera --config Release --prefix dist

# 3. Set up Python environment (first time only)
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e .

# 4. Run
python .\examples\scenarioBasicOrbit.py
```

## Troubleshooting

### "VCPKG_ROOT not set" or wrong vcpkg path

Ensure `VCPKG_ROOT` is set as a **system** environment variable pointing to your
own vcpkg installation (`C:\vcpkg`), not the VS-bundled one. VS Developer PowerShell
overwrites `VCPKG_ROOT` with its bundled path — the build script reads from the
system variable to work around this.

### vcpkg "Unable to find a valid Visual Studio instance"

Occurs on ARM64 Windows when vcpkg can't locate the full VS installation. Fix:
1. Ensure you installed the full VS Community/Professional edition (not Build Tools only)
2. Set `VCPKG_VISUAL_STUDIO_PATH` as a system environment variable (see section 2)

### `$env:INCLUDE` is empty in terminal

The VS Developer environment is not initialized. Either:
- Use the "PowerShell with VS" terminal profile (see section 3)
- Or run `.\build.ps1` which initializes it automatically

### Path length warnings during build

Build to a shorter path:
```powershell
.\build.ps1 -Config Release -BuildDir C:\build\xmera
cmake --install C:\build\xmera --config Release --prefix dist
```

Or move the repository to `C:\dev\xmera`.

### Python architecture mismatch

Ensure your Python architecture matches your build target:
```powershell
python -c "import platform; print(platform.machine())"
# AMD64 for x64 builds, ARM64 for arm64 builds
```

### `ImportError: cannot import name '_module'`

The `.pyd` extension module was not installed. Ensure you ran:
```powershell
cmake --install build --config Release --prefix dist
pip install -e .
```

### "Windows fatal exception: access violation" when importing xmera

This is a Debug/Release C runtime mismatch. Python is always a Release binary; loading
a Debug-built `.pyd` triggers an access violation because the two CRT instances conflict.

Fix: rebuild and reinstall with Release:
```powershell
.\build.ps1 -Config Release
cmake --install build --config Release --prefix dist
```

### `ImportError: DLL load failed while importing _spiceInterface` (or other modules)

Runtime DLLs (cspice, opencv, zmq, etc.) are not on PATH. These are installed by
`cmake --install` to `dist/lib/` and loaded automatically via `xmera/__init__.py`.

Ensure you ran:
```powershell
cmake --install build --config Release --prefix dist
```

If the error persists after reinstalling, the `dist/lib/` directory may be missing DLLs.
Check that `dist/lib/cspice.dll` exists.

### `NameError: name 'xmera' is not defined` or `AttributeError: module 'xmera' has no attribute 'architecture'`

The `xmera/__init__.py` from `cmake --install` was not installed, or `dist/` contains a
stale install from before these fixes. Rerun the install step:
```powershell
cmake --install build --config Release --prefix dist
```

## IDE Integration

### Visual Studio Code

1. Install **CMake Tools** and **C/C++** extensions
2. Configure the terminal profile as described in section 3
3. Open `C:\dev\xmera\src` as workspace
4. Select "windows-ninja" preset when prompted
5. Build using the CMake sidebar or `Ctrl+Shift+B`

### Visual Studio 2022

1. File > Open > Folder > select `C:\dev\xmera\src`
2. VS detects CMakePresets.json automatically
3. Select "windows-ninja" from the configuration dropdown
4. Build > Build All

### CLion

1. File > Open > select `C:\dev\xmera\src`
2. CLion detects CMake project automatically
3. Settings > Build > CMake > select windows-ninja preset