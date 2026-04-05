<#
.SYNOPSIS
    Build script for xmera on Windows.

.DESCRIPTION
    Configures and builds xmera using CMake and Ninja.

    Prerequisites:
    - Run from Developer PowerShell for VS 2022
    - VCPKG_ROOT environment variable set to your vcpkg installation
    - See docs/windows-setup.md for full setup instructions

.PARAMETER Config
    Build configuration: Debug or Release. Default: Debug

.PARAMETER Arch
    Target architecture: x64 or arm64. Default: auto-detect from Python

.PARAMETER Clean
    Remove the build directory before building.

.PARAMETER BuildDir
    Custom build directory path. Default: <repo>\build

.PARAMETER ConfigureOnly
    Only run CMake configure, don't build.

.EXAMPLE
    .\build.ps1
    Build Debug with default settings

.EXAMPLE
    .\build.ps1 -Config Release -BuildDir C:\build\xmera
    Build Release to a shorter path

.EXAMPLE
    .\build.ps1 -Clean -Config Release
    Clean build Release configuration
#>

[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",

    [ValidateSet("x64", "arm64", "")]
    [string]$Arch = "",

    [switch]$Clean,

    [string]$BuildDir = "",

    [switch]$ConfigureOnly
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

# Detect architecture from Python before VS init (Python works without VS environment)
if (-not $Arch) {
    try {
        $pythonArch = python -c "import platform; print(platform.machine())"
        $Arch = if ($pythonArch -eq "AMD64") { "x64" } else { "arm64" }
    } catch {
        $Arch = "x64"
    }
}

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
# can find all required toolchains (including ARM64 host tools on ARM64 Windows)
if (-not $env:VCPKG_VISUAL_STUDIO_PATH -and $env:VSINSTALLDIR) {
    $env:VCPKG_VISUAL_STUDIO_PATH = $env:VSINSTALLDIR.TrimEnd('\')
}

# Initialize VS environment if not already set
if (-not $env:INCLUDE -or -not $env:VSINSTALLDIR) {
    Write-Host "Initializing Visual Studio environment..." -ForegroundColor Yellow

    $devShellDll = "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
    $vsInstallPath = "C:\Program Files\Microsoft Visual Studio\18\Community"

    if (-not (Test-Path $devShellDll)) {
        $errors += "Could not find Visual Studio DevShell at: $devShellDll`n  Please install Visual Studio with C++ workload."
    } else {
        # Save variables that VS DevShell overwrites
        $savedVcpkgRoot = $env:VCPKG_ROOT
        $savedVcpkgVsPath = $env:VCPKG_VISUAL_STUDIO_PATH

        Import-Module $devShellDll
        # Enter-VsDevShell uses "amd64" not "x64"
        $devShellArch = if ($Arch -eq "x64") { "amd64" } else { $Arch }
        Enter-VsDevShell -VsInstallPath $vsInstallPath -SkipAutomaticLocation -Arch $devShellArch

        # Restore variables overwritten by VS DevShell
        $env:VCPKG_ROOT = $savedVcpkgRoot
        $env:VCPKG_VISUAL_STUDIO_PATH = $savedVcpkgVsPath

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

Write-Host "Configuration: $Config" -ForegroundColor Green
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

    # Convert paths to forward slashes for CMake
    $SrcDirCMake = $SrcDir -replace '\\', '/'
    $BuildDirCMake = $BuildDir -replace '\\', '/'
    $ScriptDirCMake = $ScriptDir -replace '\\', '/'
    $VcpkgRootCMake = $env:VCPKG_ROOT -replace '\\', '/'
    $PythonExeCMake = $PythonExe -replace '\\', '/'

    $cmakeArgs = @(
        "-S", $SrcDirCMake,
        "-B", $BuildDirCMake,
        "-G", "Ninja Multi-Config",
        "-DCMAKE_TOOLCHAIN_FILE=$VcpkgRootCMake/scripts/buildsystems/vcpkg.cmake",
        "-DCMAKE_MODULE_PATH=$SrcDirCMake/cmake",
        "-DVCPKG_TARGET_TRIPLET=$VcpkgTriplet",
        "-DPython3_EXECUTABLE=$PythonExeCMake",
        "-DCMAKE_INSTALL_PREFIX=$ScriptDirCMake/dist",
        "-DXMERA_MODULE_ROOTS=fswAlgorithms;simulation;moduleTemplates",
        "-DXMERA_ENABLE_GROUPS=simulation;fswAlgorithms;moduleTemplates",
        "-DXMERA_ENABLE_INTERNAL=YES",
        "-DXMERA_ENABLE_FUZZTESTS=OFF"
    )

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
Write-Host "Building ($Config)..." -ForegroundColor Yellow

& cmake --build $BuildDir --config $Config --parallel

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "  BUILD FAILED" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "  BUILD SUCCESSFUL" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Output: $BuildDir" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "  python -m venv .venv" -ForegroundColor DarkGray
Write-Host "  .\.venv\Scripts\Activate.ps1" -ForegroundColor DarkGray
Write-Host "  pip install -e ." -ForegroundColor DarkGray
Write-Host ""
