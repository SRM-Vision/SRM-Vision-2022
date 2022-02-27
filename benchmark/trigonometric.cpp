#include <cmath>
#include <ctime>
#include <cstdio>
#include "math-tools/algorithms.h"

const size_t kExperiments = 0x3f3f3f3f >> 4;
const size_t kHalfExperiments = kExperiments >> 1;
const size_t kQuarterExperiments = kHalfExperiments >> 1;

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    printf("Benchmark for trigonometric functions. Based on %u experiments.\n", (unsigned int) kExperiments);
    printf("================================================\n");

#if defined(HW_ACC_SSE2) | defined(HW_ACC_NEON)
    {
        printf("SSE2 or NEON detected. Testing accelerated sin and cos:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kQuarterExperiments; ++i) {
            v4sf input_data = {float(i), float(i), float(i), float(i)};
            volatile v4sf result_data = sin_ps(input_data);
        }
        v4sf test_data = {float(M_PI / 3), 0.f, 0.f, 0.f};
        v4sf result_data = sin_ps(test_data);
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: sin_ps(pi/3, 0, 0, 0) => %lf.\n", *(float *) &result_data);
        printf("------------------------------------------------\n");
    }
#endif

    {
        printf("Testing std::sin() using cmath:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = std::sin(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: std::sin(pi/3) = %lf.\n", std::sin(M_PI / 3));
        printf("------------------------------------------------\n");
    }

#if defined(HW_ACC_SSE2) | defined(HW_ACC_NEON)
    {
        printf("SSE2 or NEON detected. Testing accelerated tan and cot:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kQuarterExperiments; ++i) {
            v4sf input_data = {float(i), float(i), float(i), float(i)};
            volatile v4sf result_data = tan_ps(input_data);
        }
        v4sf test_data = {float(M_PI / 3), 0.f, 0.f, 0.f};
        v4sf result_data = tan_ps(test_data);
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: tan_ps(pi/3, 0, 0, 0) => %lf.\n", *(float *) &result_data);
        printf("------------------------------------------------\n");
    }
#endif

    {
        printf("Testing std::tan() using cmath:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = std::tan(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: std::tan(pi/3) = %lf.\n", std::tan(M_PI / 3));
        printf("------------------------------------------------\n");
    }

#if defined(HW_ACC_SSE2) | defined(HW_ACC_NEON)
    {
        printf("SSE2 or NEON detected. Testing accelerated atan:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kQuarterExperiments; ++i) {
            v4sf input_data = {float(i), float(i), float(i), float(i)};
            volatile v4sf result_data = atan_ps(input_data);
        }
        v4sf test_data = {float(M_PI / 3), 0.f, 0.f, 0.f};
        v4sf result_data = atan_ps(test_data);
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: atan_ps(pi/3, 0, 0, 0) => %lf.\n", *(float *) &result_data);
        printf("------------------------------------------------\n");
    }
#endif

    {
        printf("Testing std::atan() using cmath:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = std::atan(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: std::atan(pi/3) = %lf.\n", std::atan(M_PI / 3));
        printf("------------------------------------------------\n");
    }

    return 0;
}
