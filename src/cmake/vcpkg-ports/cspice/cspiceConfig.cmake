include(CMakeFindDependencyMacro)

# Resolve install layout
set(_CSPICE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

# Set variables for consumers
set(CSPICE_INCLUDE_DIR "${_CSPICE_ROOT}/include/cspice")
set(CSPICE_LIBRARY_RELEASE "${_CSPICE_ROOT}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}cspice${CMAKE_STATIC_LIBRARY_SUFFIX}")
set(CSPICE_LIBRARY_DEBUG "${_CSPICE_ROOT}/debug/lib/${CMAKE_STATIC_LIBRARY_PREFIX}cspice${CMAKE_STATIC_LIBRARY_SUFFIX}")

# Create an imported target
if(NOT TARGET cspice::cspice)
    add_library(cspice::cspice STATIC IMPORTED)
    set_target_properties(cspice::cspice PROPERTIES
        IMPORTED_LOCATION "${CSPICE_LIBRARY_RELEASE}"
        IMPORTED_LOCATION_RELEASE "${CSPICE_LIBRARY_RELEASE}"
        INTERFACE_INCLUDE_DIRECTORIES "${CSPICE_INCLUDE_DIR}"
    )
    if(EXISTS "${CSPICE_LIBRARY_DEBUG}")
        set_target_properties(cspice::cspice PROPERTIES
            IMPORTED_LOCATION_DEBUG "${CSPICE_LIBRARY_DEBUG}"
        )
    endif()
endif()

# Export variables as fallback for non-target-based usage
set(cspice_INCLUDE_DIRS "${CSPICE_INCLUDE_DIR}")
if(EXISTS "${CSPICE_LIBRARY_DEBUG}")
    set(cspice_LIBRARIES "${CSPICE_LIBRARY_DEBUG}")
else()
    set(cspice_LIBRARIES "${CSPICE_LIBRARY_RELEASE}")
endif()

unset(_CSPICE_ROOT)
