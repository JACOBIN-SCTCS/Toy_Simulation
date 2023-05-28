cmake_minimum_required(VERSION 3.10)
project(mapping_cpp)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_PREFIX_PATH "/home/depressedcoder/eigen-3.4.0")
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(mapping_cpp src/main.cpp)
target_link_libraries(mapping_cpp SDL2main SDL2 GL)
target_include_directories(mapping_cpp PUBLIC include/)
target_link_libraries(mapping_cpp Eigen3::Eigen)
target_compile_features(mapping_cpp PUBLIC c_std_99 cxx_std_17)  