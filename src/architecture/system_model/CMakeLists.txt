xmera_add_swig_module(architecture.sim_model)

target_link_libraries(architecture.sim_model PRIVATE
  Eigen3::Eigen
)


add_executable(test.architecture.sim_model
  tests/test_sys_model_task.cpp
)
target_link_libraries(test.architecture.sim_model PRIVATE
  GTest::gmock_main
  Xmera::Core
)
target_include_directories(test.architecture.sim_model PRIVATE
  "${CMAKE_SOURCE_DIR}"
)
gtest_discover_tests(test.architecture.sim_model)
