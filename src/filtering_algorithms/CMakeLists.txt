# filtering_algorithms — per-filter algorithm libraries.
#
# Each subdirectory holds a filter algorithm written against `filtering_core`
# (and Eigen) with no xmera coupling. The xmera adapter lives elsewhere
# (under `fswAlgorithms/`) and links against the algorithm library here.

add_subdirectory(flybyODuKF)
