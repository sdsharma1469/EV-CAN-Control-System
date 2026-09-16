cmake_minimum_required(VERSION 3.16)
project(EV_CAN_Control_System LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(evcan_controller
    src/vehicle_controller.cpp
)

target_include_directories(evcan_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(evcan_controller PRIVATE /W4)
else()
    target_compile_options(evcan_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(ev_can_sim
    src/main.cpp
)
target_link_libraries(ev_can_sim PRIVATE evcan_controller)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_controller
        tests/test_controller.cpp
    )
    target_link_libraries(test_controller PRIVATE evcan_controller)
    add_test(NAME controller_unit_tests COMMAND test_controller)
endif()
