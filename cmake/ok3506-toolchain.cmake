set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_ROOT "/home/forlinx/work/toolchain")
set(CMAKE_SYSROOT "${TOOLCHAIN_ROOT}/arm-buildroot-linux-gnueabihf/sysroot")
set(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}/bin/arm-buildroot-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_ROOT}/bin/arm-buildroot-linux-gnueabihf-g++")
set(CMAKE_AR "${TOOLCHAIN_ROOT}/bin/arm-buildroot-linux-gnueabihf-gcc-ar")
set(CMAKE_RANLIB "${TOOLCHAIN_ROOT}/bin/arm-buildroot-linux-gnueabihf-gcc-ranlib")
set(CMAKE_STRIP "${TOOLCHAIN_ROOT}/bin/arm-buildroot-linux-gnueabihf-strip")

set(CMAKE_FIND_ROOT_PATH
    "${CMAKE_SYSROOT}"
    "${TOOLCHAIN_ROOT}/arm-buildroot-linux-gnueabihf"
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_FLAGS_INIT "-O0 -g3")
set(CMAKE_EXE_LINKER_FLAGS_INIT "")
