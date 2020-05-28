file(GLOB SOURCES Source/*.cpp)

    add_library(DetourTileCache STATIC ${SOURCES})

add_library(RecastNavigation::DetourTileCache ALIAS DetourTileCache)

set(DetourTileCache_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(DetourTileCache PUBLIC
    "$<BUILD_INTERFACE:${DetourTileCache_INCLUDE_DIR}>"
)

target_link_libraries(DetourTileCache
    Detour
)

set_target_properties(DetourTileCache PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )


file(GLOB INCLUDES Include/*.h)
