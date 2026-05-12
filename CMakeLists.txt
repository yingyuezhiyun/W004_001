cmake_minimum_required(VERSION 3.16)

project(ok3506_demo C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(ok3506_demo
    src/main.c
)

target_compile_options(ok3506_demo PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -O0
    -g3
)

install(TARGETS ok3506_demo RUNTIME DESTINATION bin)
