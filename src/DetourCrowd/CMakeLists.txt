file(GLOB SOURCES Source/*.cpp)

    add_library(DetourCrowd STATIC ${SOURCES})

add_library(RecastNavigation::DetourCrowd ALIAS DetourCrowd)

set(DetourCrowd_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(DetourCrowd PUBLIC
    "$<BUILD_INTERFACE:${DetourCrowd_INCLUDE_DIR}>"
)

target_link_libraries(DetourCrowd
    Detour
)

set_target_properties(DetourCrowd PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )

file(GLOB INCLUDES Include/*.h)
