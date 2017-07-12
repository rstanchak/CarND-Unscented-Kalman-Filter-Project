project(UnscentedKF)

cmake_minimum_required (VERSION 2.8)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/ukf.cpp src/tools.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 



add_executable(UnscentedKF src/main.cpp ${sources})
target_link_libraries(UnscentedKF z ssl uv uWS)

if(${BUILD_MYSTUFF})
	include_directories(src)
	add_library(ukf ${sources})

	add_executable(UnscentedKF_test src/main2.cpp)
	target_link_libraries(UnscentedKF_test ukf)

	add_executable(test_UnscentedKF src/testmain.cpp ${sources})
	target_link_libraries(test_UnscentedKF ukf)

	add_subdirectory(ext/googletest)
	enable_testing()
	add_subdirectory(test)
endif()
