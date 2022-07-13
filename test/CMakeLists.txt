# Compile test for finite state machine.
file(GLOB FSM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../modules/fsm-base/*.c*)
add_executable(test-fsm ${CMAKE_CURRENT_SOURCE_DIR}/fsm.cpp ${FSM_SRC})
target_link_libraries(test-fsm ${COMMON_LIBS})

# Compile test for bullet trajectory solver.
file(GLOB TRAJ_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../modules/bullet-trajectory-solver/*.c*)
add_executable(test-traj-solver ${CMAKE_CURRENT_SOURCE_DIR}/traj-solver.cpp ${TRAJ_SRC})
target_link_libraries(test-traj-solver ${COMMON_LIBS})
