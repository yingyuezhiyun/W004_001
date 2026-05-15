cmake_minimum_required(VERSION 3.16)

project(ok3506_demo C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

file(GLOB_RECURSE OK3506_DEMO_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.c"
)

add_executable(ok3506_demo
    ${OK3506_DEMO_SOURCES}
)

target_include_directories(ok3506_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/core"
    "${CMAKE_CURRENT_SOURCE_DIR}/core/inc"
)

target_link_libraries(ok3506_demo PRIVATE m pthread)

target_compile_options(ok3506_demo PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -O0
    -g3
)

install(TARGETS ok3506_demo RUNTIME DESTINATION bin)
