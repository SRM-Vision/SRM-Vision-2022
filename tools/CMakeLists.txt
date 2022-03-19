# HikVision config generator.
add_executable(hv-cfg-gen ${CMAKE_CURRENT_SOURCE_DIR}/hv_cfg_gen.cpp)
target_link_libraries(hv-cfg-gen ${COMMON_LIBS} ${CAM_LIBS})
