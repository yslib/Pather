cmake_minimum_required(VERSION 3.12)

file(GLOB SOURCES src/*.cpp)

VM_EXTERNAL_MODULE(GIT_REPOSITORY https://github.com/cad420/VMUtils.git GIT_TAG MASTER)
VM_EXTERNAL_MODULE(GIT_REPOSITORY https://github.com/cad420/VMat.git GIT_TAG MASTER)


add_library(Pather STATIC)
target_sources(Pather PRIVATE ${SOURCES})


target_link_libraries(Pather DebugUtils Detour DetourCrowd DetourTileCache Recast)


set(Pather_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(Pather PUBLIC ${Pather_INCLUDE_DIR})

VM_TARGET_DEPENDENCY(Pather VMUtils PUBLIC)
VM_TARGET_DEPENDENCY(Pather VMat PUBLIC)


install(TARGETS Pather
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES include/*.h)
install(FILES ${INCLUDES} DESTINATION include)

