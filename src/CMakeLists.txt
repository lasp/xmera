cmake_minimum_required(VERSION 3.27)

# Workaround for Ninja + MSVC on ARM64: skip compiler tests to avoid rc.exe issues
if(CMAKE_GENERATOR MATCHES "Ninja" AND WIN32)
  set(CMAKE_C_COMPILER_WORKS TRUE)
  set(CMAKE_CXX_COMPILER_WORKS TRUE)
endif()

project(xmera LANGUAGES C CXX)

set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Make generated headers (like xmera_core_export.h) available to all targets
include_directories("${CMAKE_BINARY_DIR}")

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(MSVC)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -gdwarf-3")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
  # Windows path length workaround: use shorter object file paths
  set(CMAKE_OBJECT_PATH_MAX 200)
  # Use ninja-style short paths for intermediate directories
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT Embedded)
  # Try to use shorter intermediate directories
  set(CMAKE_USE_RELATIVE_PATHS ON)
  # Enable math constants like M_PI on Windows
  add_compile_definitions(_USE_MATH_DEFINES)

  # Flatten output directories to reduce path lengths
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

  # Use shorter intermediate directory names
  set(CMAKE_PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pdb")

  # Reduce verbosity of intermediate directory structure
  set(CMAKE_SUPPRESS_REGENERATION ON)

  # Workaround for Ninja + MSVC on ARM64: disable manifest embedding to avoid rc.exe issues
  if(CMAKE_GENERATOR MATCHES "Ninja")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /MANIFEST:NO")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /MANIFEST:NO")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
  endif()
else()
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif()

if(APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(SWIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
include(XmeraGoogleTest)

# Patch the Python3::Module interface link options to be more selective,
# so that all symbols must be resolved at link time *except* those that we know
# will be supplied by the Python environment at load time.
# This ensures that any *other* missing symbols are reported during build.
if(APPLE)
  get_property(_PYTHON_INTERFACE_LINK_OPTIONS TARGET Python3::Module PROPERTY INTERFACE_LINK_OPTIONS)
  list(REMOVE_ITEM _PYTHON_INTERFACE_LINK_OPTIONS "LINKER:-undefined,dynamic_lookup")
  list(APPEND _PYTHON_INTERFACE_LINK_OPTIONS "-Wl,@${CMAKE_SOURCE_DIR}/cmake/cpython-exported-symbols")
  set_property(TARGET Python3::Module PROPERTY INTERFACE_LINK_OPTIONS "${_PYTHON_INTERFACE_LINK_OPTIONS}")
endif()

# Check if the large Spice files are present.  If not, download from JPL site directly
set(URL_SPICE_KERNEL
        "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp"
        CACHE
        STRING
        "URL from which to download spice kernel de430.bsp dependency")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp")
  message(STATUS "File de430.bsp not found, downloading file from ${URL_SPICE_KERNEL}:")
  file(DOWNLOAD
          "${URL_SPICE_KERNEL}"
          # Download it to the build directory (in case the download gets interrupted).
          "${CMAKE_BINARY_DIR}/data/de430.bsp"
       SHOW_PROGRESS
       STATUS DOWNLOAD_RESULT
  )
  list(GET DOWNLOAD_RESULT 1 RESULT_MESSAGE)
  list(GET DOWNLOAD_RESULT 0 RESULT_CODE)
  if(NOT RESULT_CODE EQUAL 0)
    message(WARNING "Failed downloading de430.bsp with error code: ${RESULT_CODE} : ${RESULT_MESSAGE}.")
  endif()
  # Move the SPICE file into the appropriate directory.
  file(RENAME "${CMAKE_BINARY_DIR}/data/de430.bsp" "${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp")
else()
  message(STATUS "Found Spice files.")
endif ()


include(XmeraFramework)

# Xmera core libraries
add_subdirectory(architecture)

# Modules (and associated helper libraries)
foreach(d IN LISTS XMERA_MODULE_ROOTS)
  add_subdirectory("${d}")
endforeach()

xmera_generate_messaging_init()

install(DIRECTORY utilities DESTINATION xmera
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY ../supportData DESTINATION xmera)
install(FILES xmera_package_init.py DESTINATION xmera RENAME __init__.py)
install(FILES xmera_architecture_cSysModel.py DESTINATION xmera/architecture RENAME cSysModel.py)

# Install vcpkg runtime DLLs so Python can find them at import time.
if(WIN32)
  set(_vcpkg_bin "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/bin")
  if(EXISTS "${_vcpkg_bin}")
    install(DIRECTORY "${_vcpkg_bin}/" DESTINATION xmera FILES_MATCHING PATTERN "*.dll")
  endif()
endif()
