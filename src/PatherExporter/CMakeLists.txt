cmake_minimum_required(VERSION 3.12)

file(GLOB SOURCES src/*.cpp)


add_library(PathServer SHARED)
target_sources(PathServer PRIVATE ${SOURCES})


target_link_libraries(PathServer Pather)


set(PathServer_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(Pather PUBLIC ${PathServer_INCLUDE_DIR})


install(TARGETS PathServer
        RUNTIME DESTINATION "${CMAKE_SOURCE_DIR}/../../lib"
        COMPONENT library
        )


