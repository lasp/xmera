add_library(xmera_simulator STATIC)
target_sources(xmera_simulator PRIVATE
  sim_model.cpp
)
target_link_libraries(xmera_simulator PUBLIC
  Xmera::Core
)
target_include_directories(xmera_simulator PRIVATE
  "${CMAKE_SOURCE_DIR}"
)


xmera_add_swig_module(architecture.sim_model)
target_link_libraries(architecture.sim_model PRIVATE
  xmera_simulator
  Eigen3::Eigen
)


add_executable(test.xmera_simulator
  tests/test_sys_model_task.cpp
  tests/test_sim_model.cpp
)
target_link_libraries(test.xmera_simulator PRIVATE
  xmera_simulator
  GTest::gmock_main
)
target_include_directories(test.xmera_simulator PRIVATE
  "${CMAKE_SOURCE_DIR}"
)
gtest_discover_tests(test.xmera_simulator)
