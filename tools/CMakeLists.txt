# HikVision config generator.
add_executable(hv-cfg-gen ${CMAKE_CURRENT_SOURCE_DIR}/hv-cfg-gen.cpp)
target_link_libraries(hv-cfg-gen ${CAM_LIBS})
