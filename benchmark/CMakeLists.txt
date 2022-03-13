# Compile benchmark for different inverse sqrt realizations.
add_executable(benchmark-rsqrt ${CMAKE_CURRENT_SOURCE_DIR}/rsqrt.cpp)

# Compile benchmark for different trig functions.
add_executable(benchmark-trigonometric ${CMAKE_CURRENT_SOURCE_DIR}/trigonometric.cpp)

# Compile benchmark for SSE2 fast math.
add_executable(benchmark-sse2 ${CMAKE_CURRENT_SOURCE_DIR}/sse2.cpp)
