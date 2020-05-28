file(GLOB SOURCES Source/*.cpp)

add_library(Detour STATIC ${SOURCES})

add_library(RecastNavigation::Detour ALIAS Detour)

set(Detour_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(Detour PUBLIC
    "$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
)

set_target_properties(Detour PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )
