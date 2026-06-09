# flybyODuKF_algorithm — pure C++ implementation of the flyby OD SRUKF.
#
# Depends only on filteringCore (and Eigen via filteringCore's transitive
# dependency). No `architecture/messaging`, no msgPayload types, no
# SysModel. The xmera adapter at `fswAlgorithms/opticalNavigation/flybyODuKF`
# links to this target and supplies framework-specific marshalling.

add_library(flybyODuKF_algorithm STATIC
  flybyODuKFAlgorithm.cpp
)
add_library(xmera::flybyODuKF_algorithm ALIAS flybyODuKF_algorithm)

target_link_libraries(flybyODuKF_algorithm
  PUBLIC
    filteringCore
)

target_include_directories(flybyODuKF_algorithm
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_compile_features(flybyODuKF_algorithm
  PUBLIC
    cxx_std_23
)

add_subdirectory(_tests)
