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

## 3. Configure VS Code Terminal (Optional)

> **Note**: `build.ps1` automatically discovers and initializes the VS environment
> using `vswhere.exe`, so this step is optional. It is only needed if you want to use
> CMake directly from the terminal without `build.ps1`.

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

### Linking external module roots

External module directories (e.g. `ema-gnc`) need to appear under `src/` so CMake
can find them. On Linux/macOS you would use a symlink, but on Windows symlinks require
administrator privileges. Use a **junction** instead — it works without elevation:

```powershell
# Create a junction so src\ema-gnc points to your external repo
New-Item -ItemType Junction -Path "C:\dev\xmera\src\ema-gnc" -Target "C:\dev\ema-gnc"
```

The junction is transparent to CMake and git. To remove it:

```powershell
# Remove the junction (does NOT delete the target directory)
(Get-Item "C:\dev\xmera\src\ema-gnc").Delete()
```

> **Note**: Do not use `Remove-Item -Recurse` on a junction — it will delete the
> contents of the target directory. Always use `.Delete()` as shown above.

## 5. Build xmera

### Using the PowerShell build script (Recommended)

The build script handles VS environment initialization, vcpkg, CMake configuration,
building, and installation automatically. The build type (Release, RelWithDebInfo,
etc.) is determined by the CMake preset you choose — pass `-Preset <name>` to
select one.

> **Note**: Never build a `Debug`-config preset for Python use. Python on Windows
> is a Release binary and cannot load Debug `.pyd` modules — you will get either a
> fatal access violation or a `PyModuleDef_Type.tp_flags & Py_TPFLAGS_READY` assertion
> dialog at import time. For native debugging, use a preset with `RelWithDebInfo`
> (e.g., `ema-gnc-debug`) — it produces release-compatible binaries with `.pdb`
> symbols.

```powershell
cd C:\dev\xmera
.\build.ps1
```

With options:

```powershell
# Build to a shorter path (avoids path length warnings)
.\build.ps1 -BuildDir C:\build\xmera

# Clean build
.\build.ps1 -Clean

# Combine options
.\build.ps1 -Clean -BuildDir C:\build\xmera
```

### Using CMake directly

These are the individual steps that `build.ps1` runs. Use them when you want more
control or need to debug a specific stage.

#### Step 1: Initialize the VS Developer environment

CMake needs the MSVC compiler on PATH. Open a **Developer PowerShell for VS**, or
initialize the environment in an existing terminal:

```powershell
# Find your VS installation (vswhere ships with VS at a stable path)
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsInstallPath = & $vswhere -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath

# Load the VS Developer environment for x64
Import-Module "$vsInstallPath\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $vsInstallPath -SkipAutomaticLocation -Arch amd64
```

Verify it worked:
```powershell
$env:INCLUDE   # Should print MSVC include paths
cl.exe         # Should print the MSVC compiler version
```

#### Step 2: Set environment variables

```powershell
# Point to your vcpkg installation (VS DevShell may overwrite this with its bundled vcpkg)
$env:VCPKG_ROOT = [Environment]::GetEnvironmentVariable("VCPKG_ROOT", "Machine")

# Tell vcpkg/CMake where VS is installed
$env:VCPKG_VISUAL_STUDIO_PATH = $vsInstallPath

# Set the vcpkg triplet
$env:VCPKG_DEFAULT_TRIPLET = "x64-windows"
$env:VCPKG_TARGET_TRIPLET = "x64-windows"
```

#### Step 3: Configure (CMake)

```powershell
cd C:\dev\xmera

# Using the preset (recommended — all options are defined in src/CMakePresets.json)
cmake -S src --preset windows-ninja

# Or, if VIRTUAL_ENV is not set, override the Python path:
cmake -S src --preset windows-ninja -DPython3_EXECUTABLE=C:/dev/xmera/.venv/Scripts/python.exe

# Or, to use a custom build directory:
cmake -S src --preset windows-ninja -B C:/build/xmera
```

#### Step 4: Build

```powershell
cmake --build build --parallel
```

The build type (Release, RelWithDebInfo, etc.) is baked in by the preset — no
`--config` flag is needed.

#### Step 5: Install

This copies `.pyd` modules, DLLs, and support data to `dist/`. **Required** before
Python can import xmera.

```powershell
cmake --install build --prefix dist
```

#### Step 6: Set up Python and run

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e .
python .\examples\scenarioBasicOrbit.py
```

## 6. Set up Python and run

`build.ps1` automatically runs `cmake --install` after building, which copies `.pyd`
modules, DLLs, and support data to `dist/`. All you need to do is set up a Python
virtual environment:

```powershell
cd C:\dev\xmera

# Create and activate virtual environment (first time only)
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install xmera as a Python package (development mode, first time only)
pip install -e .

# Run an example
python .\examples\scenarioBasicOrbit.py
```

> If you used `build.ps1 -SkipInstall`, you must run the install step manually:
> ```powershell
> cmake --install build --config Release --prefix dist
> ```

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
| `-Config` | Debug or Release | Release |
| `-Arch` | x64 or arm64 | x64 |
| `-Clean` | Remove build directory first | False |
| `-BuildDir` | Custom build output path | `<repo>\build` |
| `-ConfigureOnly` | Only configure, don't build | False |
| `-SkipInstall` | Skip the cmake --install step | False |

### Full workflow from scratch

```powershell
# 1. Build and install
.\build.ps1 -Clean

# 2. Set up Python environment (first time only)
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e .

# 3. Run
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

The VS Developer environment is not initialized. `build.ps1` handles this automatically
using `vswhere.exe`. If you need the VS environment for manual CMake use, configure the
VS Code terminal profile (see section 3) or launch a Developer PowerShell.

### Path length warnings during build

Build to a shorter path:
```powershell
.\build.ps1 -BuildDir C:\build\xmera
```

Or move the repository to `C:\dev\xmera`.

### Python architecture mismatch

Ensure your Python architecture matches your build target:
```powershell
python -c "import platform; print(platform.machine())"
# AMD64 for x64 builds, ARM64 for arm64 builds
```

### `ImportError: cannot import name '_module'`

The `.pyd` extension module was not installed. Rebuild (which also reinstalls):
```powershell
.\build.ps1
pip install -e .
```

### "Windows fatal exception: access violation" when importing xmera

This is a Debug/Release C runtime mismatch. Python is always a Release binary; loading
a Debug-built `.pyd` triggers an access violation because the two CRT instances conflict.

Fix: rebuild with Release (the default):
```powershell
.\build.ps1 -Clean
```

### `ImportError: DLL load failed while importing _spiceInterface` (or other modules)

Runtime DLLs (cspice, opencv, zmq, etc.) are missing from `dist/lib/`. Rebuild
(which includes the install step):
```powershell
.\build.ps1
```

If the error persists, check that `dist/lib/cspice.dll` exists.

### `NameError: name 'xmera' is not defined` or `AttributeError: module 'xmera' has no attribute 'architecture'`

The `dist/` directory contains a stale install. Rebuild:
```powershell
.\build.ps1 -Clean
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