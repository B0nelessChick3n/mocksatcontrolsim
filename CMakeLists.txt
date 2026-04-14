#
#  ██████╗███╗   ███╗ █████╗ ██╗  ██╗███████╗██╗     ██╗███████╗████████╗███████╗████████╗██╗  ██╗████████╗
# ██╔════╝████╗ ████║██╔══██╗██║ ██╔╝██╔════╝██║     ██║██╔════╝╚══██╔══╝██╔════╝╚══██╔══╝╚██╗██╔╝╚══██╔══╝
# ██║     ██╔████╔██║███████║█████╔╝ █████╗  ██║     ██║███████╗   ██║   ███████╗   ██║    ╚███╔╝    ██║   
# ██║     ██║╚██╔╝██║██╔══██║██╔═██╗ ██╔══╝  ██║     ██║╚════██║   ██║   ╚════██║   ██║    ██╔██╗    ██║   
# ╚██████╗██║ ╚═╝ ██║██║  ██║██║  ██╗███████╗███████╗██║███████║   ██║   ███████║██╗██║   ██╔╝ ██╗   ██║   
#  ╚═════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚══════╝╚═╝╚══════╝   ╚═╝   ╚══════╝╚═╝╚═╝   ╚═╝  ╚═╝   ╚═╝   
#
cmake_minimum_required(VERSION 3.16)
project(RigidBodySim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable FetchContent fallback — use vcpkg exclusively
set(FETCHCONTENT_FULLY_DISCONNECTED ON)

find_package(box2d CONFIG REQUIRED)
find_package(SFML COMPONENTS Graphics Window System REQUIRED)

add_executable(rigid_body_sim
    main.cpp
    controller.cpp
)

target_include_directories(rigid_body_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(rigid_body_sim PRIVATE
    box2d::box2d
    SFML::Graphics
    SFML::Window
    SFML::System
)

if(MSVC)
    target_compile_options(rigid_body_sim PRIVATE /W3 /O2)
else()
    target_compile_options(rigid_body_sim PRIVATE -Wall -O2 -fno-fast-math)
endif()

# Copy config.ini next to the executable after every build so the path
# resolution in main() (argv[0]-relative) always finds it.
add_custom_command(TARGET rigid_body_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config.ini"
        "$<TARGET_FILE_DIR:rigid_body_sim>/config.ini"
    COMMENT "Copying config.ini to output directory"
)
