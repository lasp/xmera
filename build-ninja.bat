@echo off
REM Build script for Windows using Ninja generator
REM Ninja creates much shorter paths than MSBuild, avoiding path length issues
REM
REM Usage: build-ninja.bat [x64|arm64]
REM   Default: x64

REM Parse architecture parameter (default to arm64 for native builds)
set ARCH=%1
if "%ARCH%"=="" set ARCH=arm64

REM Validate architecture parameter
if not "%ARCH%"=="x64" if not "%ARCH%"=="arm64" (
    echo Error: Invalid architecture "%ARCH%"
    echo Usage: build-ninja.bat [x64^|arm64]
    pause
    exit /b 1
)

echo ============================================
echo Xmera Windows Build with Ninja (%ARCH%)
echo ============================================
echo.

REM Initialize build environment if not already set
if not defined INCLUDE (
    echo Initializing %ARCH% build environment...
    if "%ARCH%"=="x64" (
        REM Use native x64 toolchain (runs under emulation on ARM64 Windows)
        call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
    ) else (
        REM Use native ARM64 toolchain
        call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" arm64
    )
    if %ERRORLEVEL% NEQ 0 (
        echo Failed to initialize %ARCH% build environment
        pause
        exit /b 1
    )
    echo.
)

echo Compiler found: cl.exe
echo Ninja version:
ninja --version
echo.

REM Force VCPKG_ROOT to use our local vcpkg installation
REM (vcvarsall.bat may have set it to the VS bundled vcpkg)
set VCPKG_ROOT=C:\Users\pake0095\Documents\repositories\vcpkg
echo VCPKG_ROOT set to: %VCPKG_ROOT%
echo.

REM Set vcpkg triplet based on architecture
if "%ARCH%"=="x64" (
    set VCPKG_DEFAULT_TRIPLET=x64-windows
    set VCPKG_TARGET_TRIPLET=x64-windows
) else (
    set VCPKG_DEFAULT_TRIPLET=arm64-windows
    set VCPKG_TARGET_TRIPLET=arm64-windows
)

REM Force vcpkg to use the Visual Studio instance we initialized
set VCPKG_VISUAL_STUDIO_PATH=C:\Program Files\Microsoft Visual Studio\18\Community

echo Using vcpkg triplet: %VCPKG_DEFAULT_TRIPLET%
echo VCPKG_TARGET_TRIPLET: %VCPKG_TARGET_TRIPLET%
echo VCPKG_VISUAL_STUDIO_PATH: %VCPKG_VISUAL_STUDIO_PATH%
echo.

REM Get Python executable path
for /f "delims=" %%i in ('python -c "import sys; print(sys.executable)"') do set PYTHON_EXE=%%i
echo Using Python: %PYTHON_EXE%
echo.

REM Configure with Ninja if needed
if not exist "build\build.ninja" (
    echo Configuring with Ninja for %ARCH%...
    cd src
    cmake --preset windows-ninja ^
        -DVCPKG_TARGET_TRIPLET=%VCPKG_DEFAULT_TRIPLET% ^
        -DPython3_EXECUTABLE="%PYTHON_EXE%"
    if %ERRORLEVEL% NEQ 0 (
        echo Configuration failed!
        cd ..
        pause
        exit /b 1
    )
    cd ..
    echo.
)

REM Build
echo Building with Ninja for %ARCH%...
cmake --build build --config Debug --parallel
if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    pause
    exit /b 1
)

echo.
echo ============================================
echo Build completed successfully! (%ARCH%)
echo ============================================
pause
