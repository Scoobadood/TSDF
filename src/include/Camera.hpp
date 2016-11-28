//
//  Camera.hpp
//  KinFu
//
//  Created by Dave on 19/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef Camera_hpp
#define Camera_hpp

#include <Eigen/Dense>
#include <deque>
#include <cstdint>


class Camera {
private:
    // The intrinsic matrix
    Eigen::Matrix3f     m_k;

    // The inverse of K, precalculated
    Eigen::Matrix3f     m_k_inverse;

    // The pose of the camera in global coordinate space
    Eigen::Matrix4f     m_pose;

    // The inverse pose of the camera
    Eigen::Matrix4f     m_pose_inverse;

    /**
     * Common construction code
     */
    void init( );

public:

    /**
     * @return the default Kinect IR camera parmaeters
     */
    static Camera * default_depth_camera( ) {
        // Default kinect settings
        return new Camera{ 591.1f, 590.1f, 331.0f, 234.6f };
    }


    /**
     * Construct a camera with the given intrinsic parameters
     * @param focal_x The focal length in the x direction in pixels
     * @param focal_y The focal length in the y direction in pixels
     * @param centre_x The horizontal centre of the image in pixels
     * @param centre_y The vertical centre of the image in pixels
     */
    explicit Camera( const float focal_x, const float focal_y, const float centre_x, const float centre_y );

    /**
     * Construct a camera with the given intrinsic parameters as a 3x3 matrix of the form
     * fx  0   cx
     * 0   fy  cy
     * 0   0   1
     *
     * @param k The intrinsic paramters
     */
    Camera( const Eigen::Matrix3f & k );

    /**
     * Construct a camera given the image dimensions and field of view
     * @param image_width The width of the image in pixels
     * @param image_height The height of the image in pixels
     * @param fov_x The horizontal field of view in mm
     * @param fov_y The vertical field of view in mm
     */
    explicit Camera( const int image_width, const int image_height, const float fov_x, const float fov_y );

#pragma mark - K and Kinv
    /**
     * @return the intrinsic parameter matrix K
     */
    const Eigen::Matrix3f k() const;

    /**
     * @return the inverse of k
     */
    const Eigen::Matrix3f kinv() const;


#pragma mark - Pose
    /**
     * @return the pose of the camera as a 4x4 matrix
     */
    const Eigen::Matrix4f & pose( ) const;

    /**
    * @return the inverse pose of the camera as a 4x4 matrix
    */
    const Eigen::Matrix4f & inverse_pose( ) const;


    /**
     * @param The new pose of the camera as a 4x4 matrix
     */
    void set_pose( const Eigen::Matrix4f & pose );

    /**
     * Move the camera and set it's orientation based on
     * 7 float parameters (as provided by TUM groundtruth data)
     * @param vars 0, 1 and 2 are a translation
     * @param vars 3,4,5,6 are x,y,z, w components of a quaternion dewscribing the facing
     */
    void set_pose( float vars[7] );

    /**
     * Move the camera to the given global coordinates
     * @param world_coordinate The 3D world coordinate
     */
    void move_to( const Eigen::Vector3f & world_coordinate );

    /**
     * Move the camera to the given global coordinates
     * @param wx World X coordinate
     * @param wy World Y coordinate
     * @param wz World Z coordinate
     */
    void move_to( float wx, float wy, float wz );

    /**
     * Adjust the camera pose so that it faces the given point
     * assumes that 'up' is in the direction of the +ve Y axis
     * @param world_coordinate The 3D world coordinate
     */
    void look_at( const Eigen::Vector3f & world_coordinate );

    /**
     * Adjust the camera pose so that it faces the given point
     * assumes that 'up' is in the direction of the +ve Y axis
     * @param wx World X coordinate
     * @param wy World Y coordinate
     * @param wz World Z coordinate
     */
    void look_at( float wx, float wy, float wz );

    /**
     * @return the position of the camera as a vector
     */
    Eigen::Vector3f position(  ) const;




#pragma mark - Camera coordinate methods
    /**
     * Convert from pixel coordinates to camera image plane coordinates
     * @param image_coordinate The 2D coordinate in the image space
     * @return camera_coordinate The 2D coordinate in camera image plane
     */
    Eigen::Vector2f pixel_to_image_plane( const Eigen::Vector2i & image_coordinate ) const;

    /**
     * Convert from pixel coordinates to camera image plane coordinates
     * @param image_x The x coordinate in the image space
     * @param image_y The y coordinate in the image space
     * @return camera_coordinate The 2D coordinate in camera image plane
     */
    Eigen::Vector2f  pixel_to_image_plane( const uint16_t x, const uint16_t y ) const;

    /**
     * Convert from image plane to pixel coordinates
     * @param camera_coordinate The 2D coordinate in camera image plane
     * @return image_coordinate The 2D coordinate in the image space
     */
    Eigen::Vector2i image_plane_to_pixel( const Eigen::Vector2f & camera_coordinate ) const;

    /**
     * Convert the camera coordinate into world space
     * Multiply by pose
     * @param camera_coordinate The 3D pointin camera space
     * @return world_coordinate The 3D point in world space
     */
    Eigen::Vector3f camera_to_world( const Eigen::Vector3f & camera_coordinate ) const;

    /**
     * Convert the normal in world coords into camera coords
     * @param world_normal The 3D normal in world space
     * @return camera_normal The 3D normal camera space
     */
    Eigen::Vector3f  world_to_camera_normal( const Eigen::Vector3f & world_normal ) const;

    /**
     * Convert the global coordinates into camera space
     * Multiply by pose.inverse()
     * @param world_coordinate The 3D point in world space
     * @return camera_coordinate The 3D pointin camera space
     */
    Eigen::Vector3f world_to_camera( const Eigen::Vector3f & world_coordinate ) const;

    /**
     * Convert global coordinates into pixel coordinates
     * Multiply by pose.inverse(), then K
     * @param world_coordinate The 3D point in world space
     * @param pixel_coordinate The 2D point in pixel space
     */
    Eigen::Vector2i world_to_pixel( const Eigen::Vector3f & world_coordinate ) const;


#pragma mark - Depth map methods
    /**
     * Convert from a depth image to 3D camera space coordinates
     * @param depth_image A width x height array of uint16_t depth values
     * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
     * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
     */
    void depth_image_to_vertices_and_normals(const uint16_t * depth_image, const uint32_t width, const uint32_t height,
            Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices,
            Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const;
};

#endif /* Camera_hpp */
