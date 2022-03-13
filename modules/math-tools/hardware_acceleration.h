#ifndef HARDWARE_ACCELERATION_H_
#define HARDWARE_ACCELERATION_H_

#if defined(__x86_64__)

#include <emmintrin.h>  // SSE 2.

#endif

#if defined(__aarch64__)

#include "sse2neon/sse2neon.h"  // Translate SSE to NEON.

#endif

#if defined(__x86_64__) | defined(__aarch64__)
#define USE_SSE2

#include "sse-math/sse_math_extension.h"

#endif

#endif  // HARDWARE_ACCELERATION_H_
