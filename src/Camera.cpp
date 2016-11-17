//
//  Camera.cpp
//  KinFu
//
//  Created by Dave on 19/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <cmath>
#include <iostream>

#include "include/Camera.hpp"
#include "include/Definitions.hpp"


/**
 * Common construction code
 */
void Camera::init( ) {
    m_k_inverse = m_k.inverse();

    set_pose( Eigen::Matrix4f::Identity() );
}

/**
 * Construct a camera with the given intrinsic parameters
 * @param focal_x The focal length in the x direction in pixels
 * @param focal_y The focal length in the y direction in pixels
 * @param centre_x The horizontal centre of the image in pixels
 * @param centre_y The vertical centre of the image in pixels
 */
Camera::Camera( const float focal_x, const float focal_y, const float centre_x, const float centre_y ) {
    m_k << focal_x, 0.0f, centre_x, 0.0f, focal_y, centre_y, 0.0f, 0.0f, 1.0f;
    init( );
}

/**
 * Construct a camera with the given intrinsic parameters as a 3x3 matrix of the form
 * fx  0   cx
 * 0   fy  cy
 * 0   0   1
 *
 * @param k The intrinsic paramters
 */
Camera::Camera( const Eigen::Matrix3f & k ) {
    m_k = k;
    init( );
}

/**
 * Construct a camera given the image dimensions and field of view
 * @param image_width The width of the image in pixels
 * @param image_height The height of the image in pixels
 * @param fov_x The horizontal field of view in mm
 * @param fov_y The vertical field of view in mm
 */
Camera::Camera( const int image_width, const int image_height, const float fov_x, const float fov_y ) {
    float focal_x = -image_width / ( 2 * std::tan(fov_x / 2.0f ) );
    float focal_y = -image_height / ( 2 * std::tan(fov_y / 2.0f ) );
    m_k << -focal_x, 0.0f, (image_width / 2.0f), 0.0f, -focal_y, (image_height / 2.0f), 0.0f, 0.0f, 1.0f;

    init() ;
}

#pragma mark - Intrinsics
/**
 * @return the intrinsic parameter matrix K
 */
const Eigen::Matrix3f Camera::k() const {
    return m_k;
}


/**
 * @return the inverse of k
 */
const Eigen::Matrix3f Camera::kinv() const {
    return m_k_inverse;
}



#pragma mark - Pose
/**
 * @return the pose of the camera as a 4x4 matrix
 */
const Eigen::Matrix4f & Camera::pose( ) const {
    return m_pose;
}

/**
 * @return the inverse pose of the camera as a 4x4 matrix
 */
const Eigen::Matrix4f & Camera::inverse_pose( ) const {
    return m_pose_inverse;
}

/**
 * @param The new pose of the camera as a 4x4 matrix
 */
void Camera::set_pose( const Eigen::Matrix4f & pose ) {
    m_pose = pose;
    m_pose_inverse = m_pose.inverse();
}

/**
 * Move the camera to the given global coordinates
 * @param world_coordinate The 3D world coordinate
 */
void Camera::move_to( const Eigen::Vector3f & world_coordinate ) {
    move_to(world_coordinate.x(), world_coordinate.y(), world_coordinate.z() );
}

/**
 * Move the camera to the given global coordinates. Camera will maintain it's facing but will NOT keep
 * looking at a previous lookat point
 * @param wx World X coordinate
 * @param wy World Y coordinate
 * @param wz World Z coordinate
 */
void Camera::move_to( float wx, float wy, float wz ) {
    m_pose( 0, 3) = wx;
    m_pose( 1, 3) = wy;
    m_pose( 2, 3) = wz;
    m_pose_inverse = m_pose.inverse();

}

/**
 * Adjust the camera pose so that it faces the given point
 * assumes that 'up' is in the direction of the +ve Y axis
 * From gluLookat()
 * @param world_coordinate The 3D world coordinate
 */
void Camera::look_at( const Eigen::Vector3f & world_coordinate ) {
    using namespace Eigen;

    // Compute ray from current location to look_at point
    Vector3f cam_position = m_pose.block(0, 3, 3, 1);
    Vector3f forward = (world_coordinate - cam_position);
    forward.normalize();

    // If forward is straight down or up, make 'up' be the -vez axis otherwise it's the +ve y axis
    Vector3f up;
    if( ( forward.y() == 1.0f) || ( forward.y() == -1) ) {
        up << 0.0, 0.0, forward.y();
    } else {
        up << 0.0, 1.0, 0.0;
    }

    Vector3f side = forward.cross( up );
    side.normalize();

    up = side.cross( forward );

    m_pose(0,0) = side.x();
    m_pose(1,0) = side.y();
    m_pose(2,0) = side.z();
    m_pose(3,0) = 0.0f;

    m_pose(0,1) = up.x();
    m_pose(1,1) = up.y();
    m_pose(2,1) = up.z();
    m_pose(3,1) = 0.0f;

    m_pose(0,2) = -forward.x();
    m_pose(1,2) = -forward.y();
    m_pose(2,2) = -forward.z();
    m_pose(3,2) = 0.0f;

    m_pose(3,3) = 1.0f;

    m_pose_inverse = m_pose.inverse();
}

/**
 * Move the camera and set it's orientation based on
 * 7 float parameters (as provided by TUM groundtruth data)
 * @param vars 0, 1 and 2 are a translation
 * @param vars 3,4,5,6 are x,y,z, w components of a quaternion dewscribing the facing
 */
void Camera::set_pose( float vars[7] ) {
    Eigen::Vector3f tx { vars[0], vars[1], vars[2]};
    tx = tx * 1000.0f;

    Eigen::Quaternionf qq { vars[6], vars[3], vars[4], vars[5] };

    Eigen::Matrix3f r = qq.toRotationMatrix();
    Eigen::Matrix<float,1,3> c { 0.0, 0.0, -1.0 };

    c = c * r;

    Eigen::Vector3f lx = tx + ( 1000 * c.transpose() );

    move_to( tx.x(), tx.y(), tx.z() );
    look_at( lx.x(), lx.y(), lx.z() );
}



/**
 * Adjust the camera pose so that it faces the given point
 * assumes that 'up' is in the direction of the +ve Y axis
 * @param wx World X coordinate
 * @param wy World Y coordinate
 * @param wz World Z coordinate
 */
void Camera::look_at( float wx, float wy, float wz ) {
    look_at( Eigen::Vector3f { wx, wy, wz } );
}



/**
 * @return the position of the camera as a vector
 */
Eigen::Vector3f Camera::position(  ) const {
    return m_pose.block( 0, 3, 3, 1 );
}


#pragma mark - Camera coordinate methods
/**
 * Convert from pixel coordinates to camera image plane coordinates
 * @param image_coordinate The 2D coordinate in the image space
 * @return camera_coordinate The 2D coordinate in camera image plane
 */
Eigen::Vector2f Camera::pixel_to_image_plane( const Eigen::Vector2i & image_coordinate ) const {
    using namespace Eigen;

    return pixel_to_image_plane(image_coordinate.x(), image_coordinate.y() );
}

/**
 * Convert from pixel coordinates to camera image plane coordinates
 * @param image_x The x coordinate in the image space
 * @param image_y The y coordinate in the image space
 * @return camera_coordinate The 2D coordinate in camera image plane
 */
Eigen::Vector2f Camera::pixel_to_image_plane( const uint16_t x, const uint16_t y ) const {
    using namespace Eigen;

    // Multiply the (homgeous) point by the intrinsic matrix
    Vector3f homogenous_coordinate { static_cast<float>(x), static_cast<float>(y), 1.0f };
    Vector3f homogenous_camera_coordinate = m_k_inverse * homogenous_coordinate;

    Vector2f camera_coordinate = homogenous_camera_coordinate.block(0,0,2,1) / homogenous_camera_coordinate[2];
    return camera_coordinate;
}

/**
 * Convert from image plane to pixel coordinates
 * @param camera_coordinate The 2D coordinate in camera image plane
 * @return image_coordinate The 2D coordinate in the image space
 */
Eigen::Vector2i Camera::image_plane_to_pixel( const Eigen::Vector2f & camera_coordinate ) const {
    using namespace Eigen;

    Vector3f cam_h { camera_coordinate.x(), camera_coordinate.y(), 1.0f };
    Vector3f image_h = m_k * cam_h;
    Eigen::Vector2i image_coordinate;
    image_coordinate.x( ) = std::round( image_h.x() );
    image_coordinate.y( ) = std::round( image_h.y() );
    return image_coordinate;
}



/**
 * Convert the camera coordinate into world space
 * Multiply by pose
 * @param camera_coordinate The 3D pointin camera space
 * @return world_coordinate The 3D point in world space
 */
Eigen::Vector3f Camera::camera_to_world( const Eigen::Vector3f & camera_coordinate ) const {
    using namespace Eigen;

    Vector4f cam_h { camera_coordinate.x(), camera_coordinate.y(), camera_coordinate.z(), 1.0f};
    Vector4f world_h = m_pose * cam_h;

    return world_h.block(0, 0, 3, 1) / world_h.w();
}

/**
 * Convert the normal in world coords into camera coords
 * @param world_normal The 3D normal in world space
 * @return camera_normal The 3D normal camera space
 */
Eigen::Vector3f Camera::world_to_camera_normal( const Eigen::Vector3f & world_normal ) const {
    return m_pose_inverse.block( 0, 0, 3, 3) * world_normal;
}

/**
 * Convert the global coordinates into camera space
 * Multiply by pose.inverse()
 * @param world_coordinate The 3D point in world space
 * @return camera_coordinate The 3D pointin camera space
 */
Eigen::Vector3f Camera::world_to_camera( const Eigen::Vector3f & world_coordinate ) const {
    using namespace Eigen;

    Vector4f world_h { world_coordinate.x(), world_coordinate.y(), world_coordinate.z(), 1.0f};
    Vector4f cam_h = m_pose_inverse * world_h;

    return cam_h.block(0, 0, 3, 1) / cam_h[3];
}

/**
 * Convert global coordinates into pixel coordinates
 * Multiply by pose.inverse(), then K
 * @param world_coordinate The 3D point in world space
 * @return pixel_coordinate The 2D point in pixel space
 */
Eigen::Vector2i Camera::world_to_pixel( const Eigen::Vector3f & world_coordinate ) const {
    using namespace Eigen;

    // To cam coordinate space
    Vector4f cam_coordinate_h = m_pose_inverse * Vector4f { world_coordinate.x(), world_coordinate.y(), world_coordinate.z(), 1.0 };
    Vector3f cam_coordinate = cam_coordinate_h.block(0,0,3,1) / cam_coordinate_h[3];

    // Push into camera image
    Vector3f cam_image_coordinate = m_k * cam_coordinate;
    cam_image_coordinate = cam_image_coordinate / cam_image_coordinate  [2];

    // To pixel space

    // Round and store
    Eigen::Vector2i pixel_coordinate;
    pixel_coordinate.x() = std::round(cam_image_coordinate[0]);
    pixel_coordinate.y() = std::round(cam_image_coordinate[1]);

    return pixel_coordinate;
}

#pragma mark - Depth map methods
/**
 * Convert from a depth image to 3D camera space coordinates
 * @param depth_image A width x height array of uint16_t depth values
 * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
 * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
 */
void Camera::depth_image_to_vertices_and_normals(const uint16_t * depth_image, const uint32_t width, const uint32_t height, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const {
    using namespace Eigen;

    // Allocate storage for vertx and norms
    vertices.resize( 3, width * height );
    normals.resize( 3, width * height );

    // Run from bottom right corner so we can create normal sin the same pass
    int32_t src_idx = (width * height) - 1;
    for( int16_t y=height-1; y>=0; y-- ) {
        for( int16_t x=width-1; x >= 0; x-- ) {

            // Vertex and normal for this pixel
            Vector3f vertex;
            Vector3f normal { 0, 0, 0 };

            // If this is a valid depth
            uint16_t depth = depth_image[src_idx];
            if( depth != 0 ) {

                // Back project the point into camera 3D space using D(x,y) * Kinv * (x,y,1)T
                Vector2f cam_point = pixel_to_image_plane( x, y );
                vertex = Vector3f { cam_point.x(), cam_point.y(), 1.0f } * depth;

                // Compute normal as v(y,x+1)-v(y,x) cross v(y+1, x)-v(y, x )
                if( (y < static_cast<int16_t>(height - 1 )) && ( x < static_cast<int16_t>(width - 1 ) ) ) {
                    // We have adjacent vertex to right and below that we can extract
                    // Vector[0] is the element to the right of this one
                    // Vector[width] is the element below
                    Vector3f right_neighbour { vertices(0, src_idx + 1),     vertices(1, src_idx + 1),     vertices(2, src_idx + 1) };
                    Vector3f below_neighbour { vertices(0, src_idx + width), vertices(1, src_idx + width), vertices(2, src_idx + width) };

                    // If they are both not BAD
                    if( ( right_neighbour != BAD_VERTEX) && ( below_neighbour != BAD_VERTEX ) ) {
                        right_neighbour -= vertex;
                        below_neighbour -= vertex;

                        // Compute cross product for normal
                        normal = right_neighbour.cross( below_neighbour ).normalized();
                    }
                }
            } else {
                vertex = BAD_VERTEX;
            }

            // Store
            for( int i=0; i<3; i++ ) {
                vertices(i, src_idx) = vertex[i];
                normals(i, src_idx) = normal[i];
            }

            src_idx--;
        }
    }
}

