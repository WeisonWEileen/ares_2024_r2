#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/simd_intrinsics.hpp"
using namespace cv;

int main(int argc, char **argv){
    float a[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    float b[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    float c[16];

    #if (CV_SIMD512 == 1) // 如果支持512位宽度
    printf("CV_SIMD512 is defined.\n");
    for (int i = 0; i < 16; i += 8)
    {
        v_float32x16 va = v512_load(a + i);
        v_float32x16 vb = v512_load(b + i);
        v_float32x16 vc = va * vb;
        v_store(c + i, vc);
        }
    #elif(CV_SIMD256 == 1) //如果支持256位宽度
    printf("CV_SIMD256 is defined.\n");
    for (int i = 0; i < 16; i += 8)
    {
        v_float32x8 va = v256_load(a + i);
        v_float32x8 vb = v256_load(b + i);
        v_float32x8 vc = va * vb;
        v_store(c + i, vc);
        }
    #elif(CV_SIMD128 == 1) //如果支持128位宽度
    printf("CV_SIMD128 is defined.\n");
    for (int i = 0; i < 16; i += 4)
    {
        v_float32x4 va = v_load(a + i);
        v_float32x4 vb = v_load(b + i);
        v_float32x4 vc = va * vb;
        v_store(c + i, vc);
        }
    #else
    print("CV_SIMD is not defined\n") for (int i = 0; i < 16; i++)
    {
    c[i] = a[i] * b[i];
    }
    #endif


    //print the result
    for(int i=0 ; i<16 ; i++){
        printf("%d:%g\n",i,c[i]);
    }
    }

