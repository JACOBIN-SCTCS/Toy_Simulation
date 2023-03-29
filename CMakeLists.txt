cmake_minimum_required(VERSION 3.10)
project(mapping_cpp)

add_executable(mapping_cpp src/main.cpp)
target_link_libraries(mapping_cpp SDL2main SDL2 GL)
target_include_directories(mapping_cpp PUBLIC include/)