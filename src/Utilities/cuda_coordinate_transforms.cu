#include "../include/cuda_coordinate_transforms.hpp"

__device__
/**
 * Convert global coordinates into pixel coordinates
 * Multiply by pose.inverse(), then K
 * @param world_coordinate The 3D point in world space
 * @return pixel_coordinate The 2D point in pixel space
 */
int3 world_to_pixel( const float3 & world_coordinate, const Mat44 & inv_pose, const Mat33 & k ) {
	float3 cam_coordinate;
	cam_coordinate.x = inv_pose.m11 * world_coordinate.x + inv_pose.m12 * world_coordinate.y + inv_pose.m13 * world_coordinate.z + inv_pose.m14;
	cam_coordinate.y = inv_pose.m21 * world_coordinate.x + inv_pose.m22 * world_coordinate.y + inv_pose.m23 * world_coordinate.z + inv_pose.m24;
	cam_coordinate.z = inv_pose.m31 * world_coordinate.x + inv_pose.m32 * world_coordinate.y + inv_pose.m33 * world_coordinate.z + inv_pose.m34;


	// Push into camera image
	float3 image_coordinate;
	image_coordinate.x = k.m11 * cam_coordinate.x + k.m12 * cam_coordinate.y + k.m13 * cam_coordinate.z;
	image_coordinate.y = k.m21 * cam_coordinate.x + k.m22 * cam_coordinate.y + k.m23 * cam_coordinate.z;
	image_coordinate.z = k.m31 * cam_coordinate.x + k.m32 * cam_coordinate.y + k.m33 * cam_coordinate.z;

	// Round and store
	int3 pixel_coordinate;
	pixel_coordinate.x = round( image_coordinate.x / image_coordinate.z);
	pixel_coordinate.y = round( image_coordinate.y / image_coordinate.z);
    pixel_coordinate.z = 1;

	return pixel_coordinate;
}

/**
 * Convert pixel coordinates into world coordinates via a depth
 * Multiply by k.inverse(), project by depth, then pose
 * @param pixel The 2d pixel coordinate
 * @param depth The depth value of
 * @return world coordinate in 3D space
 */
__device__
float3 pixel_to_world( const int3 pixel, const Mat44 & pose, const Mat33 & inv_k, float depth ) {
	// From image to camera plane
	float3 image_plane_coords {
		inv_k.m11 * pixel.x + inv_k.m12 * pixel.y + inv_k.m13,
		inv_k.m21 * pixel.x + inv_k.m22 * pixel.y + inv_k.m23,
		inv_k.m31 * pixel.x + inv_k.m32 * pixel.y + inv_k.m33
	};

	// TODO
	// The actual calc here should be to scale the cam coords so tat Z == depth
	// There's an implicit assumption here that the Z coord is 1 which may not be true
	float3 cam_coords = f3_mul_scalar( depth, image_plane_coords);


	// Back to world coords
	float3 world_coords;
	world_coords.x = pose.m11 * cam_coords.x + pose.m12 * cam_coords.y + pose.m13 * cam_coords.z + pose.m14;
	world_coords.y = pose.m21 * cam_coords.x + pose.m22 * cam_coords.y + pose.m23 * cam_coords.z + pose.m24;
	world_coords.z = pose.m31 * cam_coords.x + pose.m32 * cam_coords.y + pose.m33 * cam_coords.z + pose.m34;

	float w = pose.m41 * cam_coords.x + pose.m42 * cam_coords.y + pose.m43 * cam_coords.z + pose.m44;

	world_coords.x /= w;
	world_coords.y /= w;
	world_coords.z /= w;

	return world_coords;
}

/**
 * Convert a camera coordinate into pixel space in the camera image
 * by multiplying by K - the intrinsice matrix - then dividing through by
 * z coordinate to project down.
 *
 * @param camera_coordinate The camera coordinate to convert
 * @param K The camera intrinsic matrix
 * @return a 2D integral pixel value (u,v) z value is 1
 */
__device__
int3 camera_to_pixel( const float3& camera_coordinate, const Mat33& k ) {
    // Project down to image plane
    float image_x = camera_coordinate.x / camera_coordinate.z;
    float image_y = camera_coordinate.y / camera_coordinate.z;

    // And into pixel plane
    image_x = (k.m11 * image_x) + ( k.m12 * image_y) + (k.m13);
    image_y = (k.m21 * image_x) + ( k.m22 * image_y) + (k.m23);

    // Nature of k is that last row is 0 0 1 so no need to de-homogenise

    // Adjust by cam intrinsics
    int3 pixel_coordinate {
        static_cast<int>(round( image_x ) ),
        static_cast<int>(round( image_y ) ),
        1
    };

    return pixel_coordinate;
}

/**
 * Convert a world coordinate into camera space by
 * by multiplying by pose matrix inverse
 * @param world_coordinate The world coordinate to convert
 * @param inv_pose The 4x4 inverse pose matrix
 * @return a 3D coordinate in camera space
 */
__device__
float3 world_to_camera( const float3& world_coordinate, const Mat44& inv_pose ) {
    float3 cam_coordinate {
        (inv_pose.m11 * world_coordinate.x ) + (inv_pose.m12 * world_coordinate.y) + (inv_pose.m13 * world_coordinate.z) + inv_pose.m14,
        (inv_pose.m21 * world_coordinate.x ) + (inv_pose.m22 * world_coordinate.y) + (inv_pose.m23 * world_coordinate.z) + inv_pose.m24,
        (inv_pose.m31 * world_coordinate.x ) + (inv_pose.m32 * world_coordinate.y) + (inv_pose.m33 * world_coordinate.z) + inv_pose.m34
    };
    float w = (inv_pose.m41 * world_coordinate.x ) + (inv_pose.m42 * world_coordinate.y) + (inv_pose.m43 * world_coordinate.z) + inv_pose.m44;

    cam_coordinate.x /= w;
    cam_coordinate.y /= w;
    cam_coordinate.z /= w;

    return cam_coordinate;
}


/**
 * Convert pixel coordinates into camera coordinates via a depth
 * Multiply by k.inverse(), project by depth
 * @param pixel The 2d pixel coordinate
 * @param depth The depth value of
 * @return world coordinate in 3D space
 */
__device__
float3 pixel_to_camera( const int3 pixel, const Mat33 & inv_k, float depth ) {
	// From image to camera plane
	float3 image_plane_coords {
		inv_k.m11 * pixel.x + inv_k.m12 * pixel.y + inv_k.m13,
		inv_k.m21 * pixel.x + inv_k.m22 * pixel.y + inv_k.m23,
		inv_k.m31 * pixel.x + inv_k.m32 * pixel.y + inv_k.m33
	};

	// The actual calc here scales the cam coords so that Z == depth
	// Usually, image_plane_coords.z==1 though this is not guaranteed
	float scale_factor = depth / image_plane_coords.z;
	float3 cam_coords = f3_mul_scalar( scale_factor, image_plane_coords);

	return cam_coords;
}
