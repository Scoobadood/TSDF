#ifndef CU_COMMON_H
#define CU_COMMON_H

#include <string>

static inline int divUp(int total, int grain) {
    return (total + grain - 1) / grain;
}

typedef struct {
    float m11, m21, m31, m41;
    float m12, m22, m32, m42;
    float m13, m23, m33, m43;
    float m14, m24, m34, m44;
} Mat44;

typedef struct {
    float m11, m21, m31;
    float m12, m22, m32;
    float m13, m23, m33;
} Mat33;

void check_cuda_error( const std::string& message, const cudaError_t err );


#endif