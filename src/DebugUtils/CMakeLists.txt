file(GLOB SOURCES Source/*.cpp)

    add_library(DebugUtils STATIC ${SOURCES})

add_library(RecastNavigation::DebugUtils ALIAS DebugUtils)

set(DebugUtils_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(DebugUtils PUBLIC
    "$<BUILD_INTERFACE:${DebugUtils_INCLUDE_DIR}>"
)

target_link_libraries(DebugUtils
    Recast
    Detour
    DetourTileCache
)

set_target_properties(DebugUtils PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )

