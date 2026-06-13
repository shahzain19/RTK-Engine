cmake_minimum_required(VERSION 3.10)
project(rtk_engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(rtk_engine src/main.cpp)
target_link_libraries(rtk_engine Eigen3::Eigen)

add_executable(test_rtcm src/test_rtcm.cpp)
target_link_libraries(test_rtcm Eigen3::Eigen)

add_executable(demo_phase2 src/demo_phase2.cpp)
target_link_libraries(demo_phase2 Eigen3::Eigen)
