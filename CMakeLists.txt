cmake_minimum_required(VERSION 3.10)
project(srm-vision-2022)

# Custom executable name.
set(EXECUTABLE_NAME srm-vision)

# Custom library path.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(TensorRT_PATH /opt/TensorRT-7.2.3.4)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_PATH /usr/src/tensorrt)
    set(TensorRT_LIB_PATH /usr/lib/aarch64-linux-gnu)
endif ()
set(MVS_PATH /opt/MVS)
set(CUDA_PATH /usr/local/cuda)
set(DH_CAM_PATH /opt/Galaxy_camera)

# Use g++-8 and C++ 17 standard for arm64 compatibility.
set(CMAKE_CXX_COMPILER /usr/bin/g++-8)
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    add_compile_options(-msse2)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options()
endif ()

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_compile_options(-O0)
elseif (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(
            # -O3  # for optimization issues
            -Ofast -march=native -flto
    )
endif ()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
file(GLOB CUDA_LIBS ${CUDA_PATH}/lib64/libcu*.so)

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
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(fmt REQUIRED)

include_directories(${MVS_PATH}/include)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/aarch64/*.so)
endif ()
include_directories(${DH_CAM_PATH}/inc)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/x86_64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/armv8/*.so)
endif ()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Ceres 2 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    include_directories(/usr/include/x86_64-linux-gnu)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(/usr/include/aarch64-linux-gnu)
endif ()

set(NVIDIA_LIBS
        ${TensorRT_LIBS}
        ${CUDA_LIBS}
        nvonnxparser
        cuda
        )

set(CAM_LIBS
        ${MVS_LIBS}
        ${DH_LIBS}
        )

set(COMMON_LIBS
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        ${CERES_LIBRARIES}
        fmt::fmt
        stdc++fs
        gflags
        )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/*/*.c*)
file(GLOB THIRD_PARTY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*/*.c*)

add_executable(${EXECUTABLE_NAME} main.cpp ${SRC} ${THIRD_PARTY_SRC} ${TensorRT_SOURCE})
target_link_libraries(${EXECUTABLE_NAME} ${NVIDIA_LIBS} ${CAM_LIBS} ${COMMON_LIBS} ${InferenceEngine_LIBRARIES})

if (NOT DEFINED DO_NOT_BUILD_COMPONENTS)
    add_subdirectory(benchmark)
    add_subdirectory(test)
    add_subdirectory(tools)
endif ()
