cmake_minimum_required(VERSION 3.12)

add_library(PyRecast SHARED)
target_sources(PyRecast recast.cpp)
target_link_libraries(PyRecast Detour Recast)
