# MVS.
set(MVS_PATH /opt/MVS)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/aarch64/*.so)
endif ()

# DH Galaxy Camera.
set(DH_CAM_PATH /opt/Galaxy_camera)
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/x86_64/*.so)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/armv8/*.so)
endif ()

# OpenCV.
find_package(OpenCV 4 REQUIRED)

# FMT.
find_package(fmt REQUIRED)

# Eigen 3.
find_package(Eigen3 REQUIRED)

# Ceres-Solver.
find_package(Ceres 2 REQUIRED)

# INCLUDE_DIRS for main CMakeLists
set(Visual_INCLUDE_DIRS
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${DH_CAM_PATH}/inc
        ${MVS_PATH}/include
        )

# LIBS for main CMakeLists
set(Visual_LIBS
        ${MVS_LIBS}
        ${DH_LIBS}
        ${OpenCV_LIBS}
        fmt::fmt
        ${CERES_LIBRARIES}
        )


find_package_handle_standard_args(
        Visual
        DEFAULT_MSG
        Visual_INCLUDE_DIRS
        Visual_LIBS
)