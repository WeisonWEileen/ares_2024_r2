#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/simd_intrinsics.hpp>
using namespace cv;

int main(int argc, char **argv)
{
#ifdef CV_SIMD
    printf("CV_SIMD is : " CVAUX_STR(CV_SIMD) "\n");
    printf("CV_SIMD_WIDTH is : " CVAUX_STR(CV_SIMD_WIDTH) "\n");
    printf("CV_SIMD128 is : " CVAUX_STR(CV_SIMD128) "\n");
    printf("CV_SIMD256 is : " CVAUX_STR(CV_SIMD256) "\n");
    printf("CV_SIMD512 is : " CVAUX_STR(CV_SIMD512) "\n");
#else
    printf("CV_SIMD is NOT defined\n");
#endif

#ifdef CV_SIMD
    printf("sizeof(v_uint8) = %d\n", (int)sizeof(v_uint8));
    printf("sizeof(v_int32) = %d\n", (int)sizeof(v_int32));
    printf("sizeof(v_float32) = %d\n", (int)sizeof(v_float32));
#endif

    return 0;
}