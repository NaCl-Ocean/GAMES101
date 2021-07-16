cmake_minimum_required (VERSION 2.8.12)
project (Transformation)

# set(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3)

# find_package(Eigen3 REQUIRED)
include_directories(/usr/local/include/eigen3)


add_executable (Transformation main.cpp)
