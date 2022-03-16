# GFlags
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})

# INCLUDE_DIRS for main CMakeLists
set(Utils_INCLUDE_DIRS
        ${gflags_INCLUDE_DIR}
        )

# LIBS for main CMakeLists
set(Utils_LIBS
        gflags
        )

find_package_handle_standard_args(
        Utils
        DEFAULT_MSG
        Utils_INCLUDE_DIRS
        Utils_LIBS
)