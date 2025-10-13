cmake_minimum_required(VERSION 3.27)
project(xmera LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(SWIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest REQUIRED)
enable_testing()

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
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp")
  message(STATUS "File de430.bsp not found, downloading file:")
  file(DOWNLOAD
          "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp"
          # Download it to the build directory (in case the download gets interrupted).
          "${CMAKE_BINARY_DIR}/data/de430.bsp"
       SHOW_PROGRESS
       STATUS DOWNLOAD_RESULT
  )
  list(GET DOWNLOAD_RESULT 1 RESULT_MESSAGE)
  list(GET DOWNLOAD_RESULT 0 RESULT_CODE)
  if(NOT RESULT_CODE EQUAL 0)
    message(FATAL_ERROR "Failed downloading de430.bsp with error code: ${RESULT_CODE} : ${RESULT_MESSAGE}.")
  endif()
  # Move the SPICE file into the appropriate directory.
  file(RENAME "${CMAKE_BINARY_DIR}/data/de430.bsp" "${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp")
endif()


include(XmeraFramework)

# Xmera core libraries
add_subdirectory(libxmera)
add_subdirectory(architecture)

# Modules (and associated helper libraries)
foreach(d IN LISTS XMERA_MODULE_ROOTS)
  add_subdirectory("${d}")
endforeach()

xmera_generate_messaging_init()

install(DIRECTORY utilities DESTINATION Basilisk)
install(DIRECTORY ../supportData DESTINATION Basilisk)
