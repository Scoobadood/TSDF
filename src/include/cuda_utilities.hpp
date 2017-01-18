#ifndef CU_COMMON_H
#define CU_COMMON_H

#include <string>
#include "vector_types.h"
#include "vector_functions.h"

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
 * Perform per element division of f1 by v2
 * @param f1 the first float3 (numerators)
 * @param i2 The second float3 (denominators)
 * @return f1 ./ f2
 */
__device__ __host__ __forceinline__
float3 f3_div_elem( const float3& f, const dim3& i ) {
    return make_float3( f.x/i.x, f.y/i.y, f.z/i.z );
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
__device__ __host__ __forceinline__
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

/**
 * Perform a matrix multiplication
 * @param mat33 The matrix
 * @param vec3 The vector
 * @return an output vector
 */
__device__ __forceinline__
float3 m3_i3_mul( const Mat33& mat, const int3& vec ) {
    float3 result;
    result.x = mat.m11 * vec.x + mat.m12 * vec.y + mat.m13 * vec.z;
    result.y = mat.m21 * vec.x + mat.m22 * vec.y + mat.m23 * vec.z;
    result.z = mat.m31 * vec.x + mat.m32 * vec.y + mat.m33 * vec.z;
    return result;
}

/**
 * Project down into pixel coordinates
 * given the inv_pose and K matrices
 * plus a point in world coords
 */
__device__ __forceinline__
int3 world_to_pixel( const Mat44& inv_pose, const Mat33& k, const float3& point ) {
    float3 cam_point;
    cam_point.x = inv_pose.m11 * point.x + inv_pose.m12 * point.y + inv_pose.m13 * point.z + inv_pose.m14;
    cam_point.y = inv_pose.m21 * point.x + inv_pose.m22 * point.y + inv_pose.m23 * point.z + inv_pose.m24;
    cam_point.z = inv_pose.m31 * point.x + inv_pose.m32 * point.y + inv_pose.m33 * point.z + inv_pose.m34;
    float w = inv_pose.m41 * point.x + inv_pose.m42 * point.y + inv_pose.m43 * point.z + inv_pose.m44;
    cam_point.x /= w;
    cam_point.y /= w;
    cam_point.z /= w;

    float3 image_point {
        cam_point.x / cam_point.z,
        cam_point.y / cam_point.z,
        1.0f
    };
    int3 pixel;
    pixel.x = static_cast<uint>( round( k.m11 * image_point.y + k.m12 * image_point.z + k.m13 ) );
    pixel.y = static_cast<uint>( round( k.m21 * image_point.y + k.m22 * image_point.z + k.m23 ) );
    pixel.z = static_cast<uint>( round( k.m31 * image_point.y + k.m32 * image_point.z + k.m33 ) ); // Should be 1

    return pixel;
}

__device__
uint8_t atomicIncUint8( uint8_t* address );

void cudaSafeAlloc( void ** ptr, size_t sz, const std::string& purpose );
void cudaSafeFree( void * ptr, const std::string& purpose );


#endif