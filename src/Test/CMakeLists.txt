
cmake_minimum_required(VERSION 3.12)

add_executable(aTest)

target_sources(aTest PRIVATE test.cpp)
target_link_libraries(aTest Detour DebugUtils DetourCrowd DetourTileCache Recast Pather)
target_include_directories(aTest PRIVATE ${Utils_INCLUDE_DIR})

