# Compile test for bullet trajectory solver.
file(GLOB TRAJ_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../modules/trajectory-solver/*.c*)
add_executable(test-traj-solver ${CMAKE_CURRENT_SOURCE_DIR}/traj-solver.cpp ${TRAJ_SRC})
target_link_libraries(test-traj-solver ${COMMON_LIBS})
