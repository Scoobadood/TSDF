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

namespace phd {
    
    class Camera {
    private:
        // The intrinsic matrix
        Eigen::Matrix3f     m_k;
        
        // The inverse of K, precalculated
        Eigen::Matrix3f     m_k_inverse;
        
        // The pose of the camera in global coordinate space
        Eigen::Matrix4f     m_pose;
        
        /**
         * Common construction code
         */
        void init( );
        
    public:
        /**
         * Construct a camera with the given intrinsic parameters
         * @param focal_x The focal length in the x direction in pixels
         * @param focal_y The focal length in the y direction in pixels
         * @param centre_x The horizontal centre of the image in pixels
         * @param centre_y The vertical centre of the image in pixels
         */
        Camera( const float focal_x, const float focal_y, const float centre_x, const float centre_y );
        
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
        
#pragma mark - Pose
        /**
         * @return the pose of the camera as a 4x4 matrix
         */
        const Eigen::Matrix4f & pose( ) const;
        
        /**
         * @param The new pose of the camera as a 4x4 matrix
         */
        void set_pose( const Eigen::Matrix4f & pose );
        
        
#pragma mark - Camera coordinate methods
        /**
         * Convert from image plane coordinates to camera space coordinates
         * @param image_coordinate The 2D coordinate in the image space
         * @param camera_coordinate The 2D coordinate in camera image plane
         */
        void image_to_camera( const Eigen::Vector2i & image_coordinate, Eigen::Vector2f & camera_coordinate ) const;
        
        /**
         * Convert from image plane coordinates to camera space coordinates
         * @param image_x The x coordinate in the image space
         * @param image_y The y coordinate in the image space
         * @param camera_coordinate The 2D coordinate in camera image plane
         */
        void image_to_camera( const int x, const int y, Eigen::Vector2f & camera_coordinate ) const;

        /**
         * Convert from camera coordinates to image coordinates
         * @param camera_x The x coordinate in the camera plane
         * @param camera_y The y coordinate in the camera plane
         * @param image_coordinate The 2D coordinate in camera's image
         */
        void camera_to_image_plane( const float x, const float y, Eigen::Vector2i & image_coordinate ) const;

        /**
         * Convert from camera coordinates to image coordinates
         * @param camera_coordinates The point coorinates in camera plane homegenous
         * @param image_coordinate The 2D coordinate in camera's image
         */
        void camera_to_image_plane( const Eigen::Vector3f camera_coordinates, Eigen::Vector2i & image_coordinate ) const;

        /**
         * Convert the global coordinates into camera space
         * Multiply by pose.inverse()
         * @param world_coordinate The 3D point in world space
         * @param camera_coordinate The 3D pointin camera space
         */
        void world_to_camera( const Eigen::Vector3f & world_coordinate, Eigen::Vector3f & camera_coordinate ) const;

        /**
         * Convert the global coordinates into camera space
         * Multiply by pose.inverse()
         * @param world_coordinate The 3D point in world space
         * @param camera_coordinate The 3D pointin camera space
         */
        void world_to_image( const Eigen::Vector3f & world_coordinate, Eigen::Vector2i & image_coordinate ) const;
        

#pragma mark - Depth map methods
        /**
         * Convert from a depth image to 3D camera space coordinates
         * @param depth_image A width x height array of uint16_t depth values
         * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
         * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
         */
        void depth_image_to_vertices_and_normals(const uint16_t * depth_image, const uint32_t width, const uint32_t height,std::deque<Eigen::Vector3f> & vertices,std::deque<Eigen::Vector3f> & normals ) const;
        /**
         * Convert from a depth image to 3D camera space coordinates
         * @param depth_image A width x height array of uint16_t depth values
         * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
         * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
         */
        void depth_image_to_vertices_and_normals( const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> & depth_image, std::deque<Eigen::Vector3f> & vertices, std::deque<Eigen::Vector3f> & normals ) const;
    };
}
#endif /* Camera_hpp */
