#ifndef HARDWARE_ACCELERATION_H_
#define HARDWARE_ACCELERATION_H_

#include <opencv2/core/cv_cpu_dispatch.h>

#if defined(HW_ACC_SSE2)

#include <emmintrin.h>  // SSE 2.

#endif

#if defined(HW_ACC_NEON)

#include "sse2neon/sse2neon.h"  // Translate SSE to NEON.

#endif

#if defined(HW_ACC_SSE2) | defined(HW_ACC_NEON)
#define USE_SSE2

#include "sse-math/sse_math_extension.h"

#endif

#endif  // HARDWARE_ACCELERATION_H_
