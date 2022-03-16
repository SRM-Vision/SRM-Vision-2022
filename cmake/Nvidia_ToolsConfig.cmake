# Find Cuda
# Custom library path.
set(CUDA_PATH /usr/local/cuda)
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
file(GLOB CUDA_LIBS ${CUDA_PATH}/lib64/libcu*.so)

# Find TensorRT
# Custom library path.
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(TensorRT_PATH /opt/TensorRT-7.2.3.4)
    # Override TensorRT path for WSL 2.
    # set(TensorRT_PATH /usr/src/tensorrt)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_PATH /usr/src/tensorrt)
    set(TensorRT_LIB_PATH /usr/lib/aarch64-linux-gnu)
endif ()

find_path(TensorRT_INCLUDE_DIRS NvInfer.h
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES include)

find_library(TensorRT_LIBRARY_INFER nvinfer
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

find_library(TensorRT_LIBRARY_INFER_PLUGIN nvinfer_plugin
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB TensorRT_LIBS ${TensorRT_PATH}/lib/libnv*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB TensorRT_LIBS ${TensorRT_LIB_PATH}/libnv*.so)
endif ()

set(Nvidia_Tools_INCLUDE_DIRS
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_Sample_INCLUDE_DIRS}
        ${TensorRT_PATH}/samples/common
        ${CUDA_INCLUDE_DIRS}
        )

set(Nvidia_Tools_LIBS
        ${TensorRT_PATH}/lib
        ${TensorRT_LIBRARY_INFER}
        ${TensorRT_LIBRARY_INFER_PLUGIN}
        ${CUDA_LIBRARIES}
        ${CUDA_LIBS}
        nvonnxparser
        cuda
        )

set(Nvidia_Tools_SOURCE
        ${TensorRT_PATH}/samples/common/logger.cpp)

find_package_handle_standard_args(
        Nvidia_Tools
        DEFAULT_MSG
        Nvidia_Tools_INCLUDE_DIRS
        Nvidia_Tools_LIBS
        Nvidia_Tools_SOURCE
)