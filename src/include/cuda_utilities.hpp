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


/**
 * Subtract one float3 from another
 * @param f1 First float3
 * @param f2 Second float3
 * @return f1 - f2
 */
__device__ __forceinline__
float3 f3_sub( const float3& f1, const float3& f2 ) {
    return make_float3( f1.x-f2.x, f1.y-f2.y, f1.z-f2.z);
}

/**
 * Add one float3 to another
 * @param f1 First float3
 * @param f2 Second float3
 * @return f1 + f2
 */
__device__ __forceinline__
float3 f3_add( const float3& f1, const float3& f2 ) {
    return make_float3( f2.x+f1.x, f2.y+f1.y, f2.z+f1.z);
}

/**
 * Multiply a float3 by a scalar value
 * @param s The scalar
 * @param vec The float3
 */
__device__ __forceinline__
float3 f3_mul_scalar( const float& scalar, const float3& vec ) {
    return make_float3( vec.x * scalar, vec.y * scalar, vec.z * scalar );
}


/**
 * Perform per element division of f1 by v2
 * @param f1 the first float3 (numerators)
 * @param f2 The second float3 (denominators)
 * @return f1 ./ f2
 */
__device__ __forceinline__
float3 f3_div_elem( const float3& f1, const float3& f2 ) {
    return make_float3( f1.x/f2.x, f1.y/f2.y, f1.z/f2.z );
}

/**
 * Normalise a float3
 * @param vec The float3 vector
 */
__device__ __forceinline__
void f3_normalise( float3 vec ) {
    float l = sqrt( vec.x*vec.x+vec.y*vec.y+vec.z*vec.z);
    vec.x /= l;
    vec.y /= l;
    vec.z /= l;
}

/**
 * Normalise a float3
 * @param vec The float3 vector
 */
__device__ __forceinline__
float f3_norm( float3 vec ) {
    return sqrt( vec.x*vec.x+vec.y*vec.y+vec.z*vec.z);
}

/**
 * Perform a matrix multiplication
 * @param mat33 The matrix
 * @param vec3 The vector
 * @return an output vector
 */
__device__ __forceinline__
float3 m3_f3_mul( const Mat33& mat, const float3& vec ) {
    float3 result;
    result.x = mat.m11 * vec.x + mat.m12 * vec.y + mat.m13 * vec.z;
    result.y = mat.m21 * vec.x + mat.m22 * vec.y + mat.m23 * vec.z;
    result.z = mat.m31 * vec.x + mat.m32 * vec.y + mat.m33 * vec.z;
    return result;
}

#endif