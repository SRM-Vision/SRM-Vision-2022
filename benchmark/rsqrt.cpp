#include <ctime>
#include <cstdio>
#include "math-tools/algorithms.h"

using namespace algorithm;

const size_t kExperiments = 0x3f3f3f3f >> 1;
const size_t kHalfExperiments = kExperiments >> 1;

inline float BitHackInvSqrtOnceIter(float x) {
    float h = .5f * x;
    int i = *(int32_t *) &x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float *) &i;
    return x * (1.5f - (h * x * x));
}

inline float BitHackInvSqrtTwiceIter(float x) {
    float h = .5f * x;
    int i = *(int32_t *) &x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float *) &i;
    x *= (1.5f - (h * x * x));
    return x * (1.5f - (h * x * x));
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    printf("Benchmark for inverse sqrt. Based on %u experiments.\n", (unsigned int) kExperiments);
    printf("================================================\n");

#if defined(HW_ACC_SSE2)
    {
        printf("SSE2 or NEON detected. Testing accelerated inverse sqrt:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = RsqrtFloat(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: RsqrtFloat(4.f) = %lf.\n", RsqrtFloat(4.f));
        printf("------------------------------------------------\n");
    }
#endif

    {
        printf("Testing 1.f / std::sqrt() using cmath:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = 1.f / std::sqrt(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: 1.f / std::sqrt(4.f) = %lf.\n", 1.f / std::sqrt(4.f));
        printf("------------------------------------------------\n");
    }

    {
        printf("Testing 0x5f375a86 with once iteration:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = BitHackInvSqrtOnceIter(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: BitHackInvSqrtOnceIter(4.f) = %lf.\n", BitHackInvSqrtOnceIter(4.f));
        printf("------------------------------------------------\n");
    }

    {
        printf("Testing 0x5f375a86 with twice iteration:\n");
        clock_t t = clock();
        for (size_t i = 0; i < kExperiments; ++i)
            volatile float res = BitHackInvSqrtTwiceIter(float(i));
        printf("Total time: %lf seconds.\n", double(clock() - t) / CLOCKS_PER_SEC);
        printf("Simple test: BitHackInvSqrtTwiceIter(4.f) = %lf.\n", BitHackInvSqrtTwiceIter(4.f));
        printf("------------------------------------------------\n");
    }

    return 0;
}
