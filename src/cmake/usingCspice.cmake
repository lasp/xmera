find_package(cspice CONFIG REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE cspice::cspice)
