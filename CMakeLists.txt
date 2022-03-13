cmake_minimum_required(VERSION 3.10)
project(srm-vision-2022)

# Custom executable name.
set(EXECUTABLE_NAME srm-vision)

# Custom library path.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(TensorRT_PATH /opt/TensorRT-7.2.3.4)
    # Override TensorRT path for WSL 2.
    # set(TensorRT_PATH /usr/src/tensorrt)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_PATH /usr/src/tensorrt)
    set(TensorRT_LIB_PATH /usr/lib/aarch64-linux-gnu)
endif ()
set(MVS_PATH /opt/MVS)
set(CUDA_PATH /usr/local/cuda)
set(DH_CAM_PATH /opt/Galaxy_camera)

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

# CUDA.
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
file(GLOB CUDA_LIBS ${CUDA_PATH}/lib64/libcu*.so)

# TensorRT.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    find_package(TensorRT REQUIRED)
    include_directories(${TensorRT_INCLUDE_DIRS})
    include_directories(${TensorRT_Sample_INCLUDE_DIRS})
    link_directories(${TensorRT_PATH}/lib)
    file(GLOB TensorRT_LIBS ${TensorRT_PATH}/lib/libnv*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    find_package(TensorRT REQUIRED)
    include_directories(${TensorRT_INCLUDE_DIRS})
    include_directories(${TensorRT_Sample_INCLUDE_DIRS})
    link_directories(${TensorRT_PATH}/lib)
    file(GLOB TensorRT_LIBS ${TensorRT_LIB_PATH}/libnv*.so)
endif ()

# OpenCV.
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# FMT.
find_package(fmt REQUIRED)

# MVS.
include_directories(${MVS_PATH}/include)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/aarch64/*.so)
endif ()

# DH Galaxy Camera.
include_directories(${DH_CAM_PATH}/inc)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/x86_64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/armv8/*.so)
endif ()

# Eigen 3.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres-Solver.
find_package(Ceres 2 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# GFlags
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})

# Linux GNU Library.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    include_directories(/usr/include/x86_64-linux-gnu)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(/usr/include/aarch64-linux-gnu)
endif ()

# Project sources.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

# Compile project main executable file.
file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/*/*.c*)
file(GLOB THIRD_PARTY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*/*.c*)
add_executable(${EXECUTABLE_NAME} main.cpp ${SRC} ${THIRD_PARTY_SRC} ${TensorRT_SOURCE})
target_link_libraries(${EXECUTABLE_NAME}
        ${MVS_LIBS}
        ${DH_LIBS}
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        ${CERES_LIBRARIES}
        ${TensorRT_LIBS}
        ${CUDA_LIBS}
        fmt::fmt
        nvonnxparser
        cuda
        stdc++fs  # In C++17, this standard library should be manually linked.
        gflags
        )

# Compile benchmarks.
add_subdirectory(benchmark)
# Compile tests.
add_subdirectory(test)
