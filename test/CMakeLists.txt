add_executable(ukf_unittest ukf.cpp)
target_link_libraries(ukf_unittest ukf gtest_main)
add_test(ukf_unittest ukf_unittest)
