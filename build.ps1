<#
.SYNOPSIS
    Build script for xmera on Windows.

.DESCRIPTION
    Configures, builds, and installs xmera using CMake and Ninja.
    Automatically discovers Visual Studio via vswhere.exe and initializes
    the VS environment if needed.

    Prerequisites:
    - Visual Studio with C++ workload installed
    - VCPKG_ROOT environment variable set to your vcpkg installation
    - See docs/windows-setup.md for full setup instructions

.PARAMETER Arch
    Target architecture: x64 or arm64. Default: x64

.PARAMETER Clean
    Remove the build directory before building.

.PARAMETER BuildDir
    Custom build directory path. Default: <repo>\build

.PARAMETER ConfigureOnly
    Only run CMake configure, don't build.

.PARAMETER Preset
    CMake configure preset name. Default: windows-ninja.
    The preset determines the build type (Release, RelWithDebInfo, etc.) —
    pick a preset that matches what you want to build.

.PARAMETER SkipInstall
    Skip the cmake --install step after building.

.EXAMPLE
    .\build.ps1
    Build with default preset (windows-ninja)

.EXAMPLE
    .\build.ps1 -Preset ema-gnc-release -BuildDir C:\build\xmera
    Build ema-gnc-release to a shorter path

.EXAMPLE
    .\build.ps1 -Preset ema-gnc-debug -Clean
    Clean build of ema-gnc-debug (RelWithDebInfo, for native debugging)
#>

[CmdletBinding()]
param(
    [ValidateSet("x64", "arm64")]
    [string]$Arch = "x64",

    [switch]$Clean,

    [string]$BuildDir = "",

    [switch]$ConfigureOnly,

    [string]$Preset = "windows-ninja",

    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"

# Script location
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$SrcDir = Join-Path $ScriptDir "src"

if (-not $BuildDir) {
    $BuildDir = Join-Path $ScriptDir "build"
}

# Banner
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  xmera Windows Build" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Check prerequisites
$errors = @()

# Read VCPKG_ROOT from the permanent system/user environment, not the process
# environment (which VS DevShell may have overwritten with its bundled vcpkg)
$systemVcpkgRoot = [Environment]::GetEnvironmentVariable("VCPKG_ROOT", "Machine")
if (-not $systemVcpkgRoot) {
    $systemVcpkgRoot = [Environment]::GetEnvironmentVariable("VCPKG_ROOT", "User")
}
if ($systemVcpkgRoot -and (Test-Path (Join-Path $systemVcpkgRoot "vcpkg.exe"))) {
    $env:VCPKG_ROOT = $systemVcpkgRoot
}

if (-not $env:VCPKG_ROOT) {
    $errors += "VCPKG_ROOT environment variable is not set. See docs/windows-setup.md."
} elseif (-not (Test-Path (Join-Path $env:VCPKG_ROOT "vcpkg.exe"))) {
    $errors += "VCPKG_ROOT ($env:VCPKG_ROOT) does not contain vcpkg.exe"
}

# Ensure VCPKG_VISUAL_STUDIO_PATH points to the full VS installation so vcpkg
# can find all required toolchains (including ARM64 host tools on ARM64 Windows).
# This is set later by vswhere if VS init is needed; for now, fall back to VSINSTALLDIR.
if (-not $env:VCPKG_VISUAL_STUDIO_PATH -and $env:VSINSTALLDIR) {
    $env:VCPKG_VISUAL_STUDIO_PATH = $env:VSINSTALLDIR.TrimEnd('\')
}

# Initialize VS environment if not already set or if arch doesn't match
# VSCMD_ARG_TGT_ARCH is set by VsDevShell, e.g. "x64", "x86", "arm64"
$currentVsArch = $env:VSCMD_ARG_TGT_ARCH
# Normalize: some VS versions report "amd64" instead of "x64"
if ($currentVsArch -eq "amd64") { $currentVsArch = "x64" }
$needsVsInit = (-not $env:INCLUDE -or -not $env:VSINSTALLDIR)
if (-not $needsVsInit -and $currentVsArch -and $currentVsArch -ne $Arch) {
    Write-Host "VS environment is for '$currentVsArch' but target is '$Arch', reinitializing..." -ForegroundColor Yellow
    $needsVsInit = $true
}
if ($needsVsInit) {
    Write-Host "Initializing Visual Studio environment..." -ForegroundColor Yellow

    # Use vswhere.exe to find VS installation dynamically (works across editions and versions)
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsInstallPath = & $vswhere -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    }
    if (-not $vsInstallPath -or -not (Test-Path $vsInstallPath)) {
        $errors += "Could not find Visual Studio with C++ workload.`n  Install Visual Studio with 'Desktop development with C++' workload."
    } else {
        $devShellDll = Join-Path $vsInstallPath "Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
    }

    if (-not $devShellDll -or -not (Test-Path $devShellDll)) {
        $errors += "Could not find Visual Studio DevShell module.`n  Please install Visual Studio with C++ workload."
    } elseif ($errors.Count -eq 0) {
        # Save variables that VS DevShell overwrites
        $savedVcpkgRoot = $env:VCPKG_ROOT
        $savedVcpkgVsPath = $env:VCPKG_VISUAL_STUDIO_PATH

        Import-Module $devShellDll
        # Enter-VsDevShell uses "amd64" not "x64"
        $devShellArch = if ($Arch -eq "x64") { "amd64" } else { $Arch }
        Enter-VsDevShell -VsInstallPath $vsInstallPath -SkipAutomaticLocation -Arch $devShellArch

        # Restore variables overwritten by VS DevShell
        $env:VCPKG_ROOT = $savedVcpkgRoot
        $env:VCPKG_VISUAL_STUDIO_PATH = if ($savedVcpkgVsPath) { $savedVcpkgVsPath } else { $vsInstallPath }

        Write-Host "VS environment initialized." -ForegroundColor Green
        Write-Host ""
    }
}

if ($errors.Count -gt 0) {
    Write-Host "ERROR: Prerequisites not met:" -ForegroundColor Red
    foreach ($err in $errors) {
        Write-Host "  - $err" -ForegroundColor Red
    }
    Write-Host ""
    Write-Host "See docs/windows-setup.md for setup instructions." -ForegroundColor Yellow
    exit 1
}

# Set vcpkg triplet
$VcpkgTriplet = "$Arch-windows"
$env:VCPKG_DEFAULT_TRIPLET = $VcpkgTriplet
$env:VCPKG_TARGET_TRIPLET = $VcpkgTriplet

Write-Host "Preset:        $Preset" -ForegroundColor Green
Write-Host "Architecture:  $Arch" -ForegroundColor Green
Write-Host "Build Dir:     $BuildDir" -ForegroundColor Green
Write-Host "VCPKG_ROOT:    $env:VCPKG_ROOT" -ForegroundColor Green
Write-Host "Triplet:       $VcpkgTriplet" -ForegroundColor Green
Write-Host ""

# Verify tools
Write-Host "Verifying tools..." -ForegroundColor Yellow
$tools = @("cmake", "ninja", "python", "swig")
$missing = @()

foreach ($tool in $tools) {
    try {
        $null = Get-Command $tool -ErrorAction Stop
        Write-Host "  [OK] $tool" -ForegroundColor Green
    } catch {
        Write-Host "  [MISSING] $tool" -ForegroundColor Red
        $missing += $tool
    }
}

if ($missing.Count -gt 0) {
    Write-Host ""
    Write-Host "ERROR: Missing tools: $($missing -join ', ')" -ForegroundColor Red
    exit 1
}
Write-Host ""

# Get Python path
$PythonExe = (Get-Command python).Source
Write-Host "Python: $PythonExe" -ForegroundColor Green
Write-Host ""

# Clean if requested
if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "Cleaning build directory..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $BuildDir
}

# Create build directory
if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
}

# Configure
$needsConfigure = -not (Test-Path (Join-Path $BuildDir "build.ninja"))

if ($needsConfigure) {
    Write-Host "Configuring..." -ForegroundColor Yellow

    # Use the windows-ninja preset; only override what the preset can't know
    $SrcDirCMake = $SrcDir -replace '\\', '/'
    $cmakeArgs = @("-S", $SrcDirCMake, "--preset", $Preset)

    # Override build dir if non-default
    $defaultBuildDir = Join-Path $ScriptDir "build"
    if ($BuildDir -ne $defaultBuildDir) {
        $cmakeArgs += "-B", ($BuildDir -replace '\\', '/')
    }

    # Always pass the Python path — custom presets may not configure it
    $PythonExeCMake = $PythonExe -replace '\\', '/'
    $cmakeArgs += "-DPython3_EXECUTABLE=$PythonExeCMake"

    # Override triplet if non-default (preset defaults to x64-windows)
    if ($Arch -ne "x64") {
        $cmakeArgs += "-DVCPKG_TARGET_TRIPLET=$VcpkgTriplet"
        $cmakeArgs += "-DVCPKG_HOST_TRIPLET=$VcpkgTriplet"
    }

    Write-Host "cmake $($cmakeArgs -join ' ')" -ForegroundColor DarkGray
    & cmake @cmakeArgs

    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "ERROR: Configuration failed." -ForegroundColor Red
        exit 1
    }
    Write-Host ""
}

if ($ConfigureOnly) {
    Write-Host "Configure complete. Skipping build (-ConfigureOnly)." -ForegroundColor Green
    exit 0
}

# Build
Write-Host "Building..." -ForegroundColor Yellow

& cmake --build $BuildDir --parallel

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "  BUILD FAILED" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    exit 1
}

# Install
$InstallPrefix = Join-Path $ScriptDir "dist"
if (-not $SkipInstall) {
    Write-Host ""
    Write-Host "Installing to $InstallPrefix..." -ForegroundColor Yellow

    & cmake --install $BuildDir --prefix $InstallPrefix

    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "========================================" -ForegroundColor Red
        Write-Host "  INSTALL FAILED" -ForegroundColor Red
        Write-Host "========================================" -ForegroundColor Red
        exit 1
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "  BUILD SUCCESSFUL" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Output:  $BuildDir" -ForegroundColor Cyan
Write-Host "Install: $InstallPrefix" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps (first time only):" -ForegroundColor Yellow
Write-Host "  python -m venv .venv" -ForegroundColor DarkGray
Write-Host "  .\.venv\Scripts\Activate.ps1" -ForegroundColor DarkGray
Write-Host "  pip install -e ." -ForegroundColor DarkGray
Write-Host ""
Write-Host "Then run:" -ForegroundColor Yellow
Write-Host "  python .\examples\scenarioBasicOrbit.py" -ForegroundColor DarkGray
Write-Host ""
