cmake_minimum_required(VERSION 3.10)
project(srm-vision-2022)

# Custom executable name.
set(EXECUTABLE_NAME srm-vision)

# Use g++-8 and C++17 for arm64 compatibility.
set(CMAKE_CXX_COMPILER /usr/bin/g++-8)
set(CMAKE_CXX_STANDARD 17)

# Common compile options.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    add_compile_options(-msse2)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options()
endif ()

# Options for different build type.
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_compile_options(
            # Complete debugger support.
            -O0
    )
elseif (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(
            # Maximum speed.
            -O3
            -Ofast
            -flto
            # Disable this flag when error occurs.
            -march=native
    )
endif ()

# Find modules.
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Linux GNU Library.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    include_directories(/usr/include/x86_64-linux-gnu)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(/usr/include/aarch64-linux-gnu)
endif ()

set(Visual_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Visual)
include_directories(${Visual_INCLUDE_DIRS})

set(Nvidia_Tools_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Nvidia_Tools)
include_directories(${Nvidia_Tools_INCLUDE_DIRS})

set(Utils_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Utils)
include_directories(${Utils_INCLUDE_DIRS})

# Project sources.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

# Compile project main executable file.
file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/*/*.c*)
file(GLOB THIRD_PARTY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*/*.c*)
add_executable(${EXECUTABLE_NAME} main.cpp
        ${SRC}
        ${THIRD_PARTY_SRC}
        ${Nvidia_Tools_SOURCE})
target_link_libraries(${EXECUTABLE_NAME}
        ${CMAKE_THREAD_LIBS_INIT}
        stdc++fs  # In C++17, this standard library should be manually linked.
        ${Visual_LIBS}
        ${Nvidia_Tools_LIBS}
        ${Utils_LIBS}
        )

# Compile benchmarks.
add_subdirectory(benchmark)
# Compile tests.
add_subdirectory(test)
