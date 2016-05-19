//
//  Camera.cpp
//  KinFu
//
//  Created by Dave on 19/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Camera.hpp"

namespace phd {
    /**
     * Construct a camera with the given intrinsic parameters
     * @param focal_x The focal length in the x direction in pixels
     * @param focal_y The focal length in the y direction in pixels
     * @param centre_x The horizontal centre of the image in pixels
     * @param centre_y The vertical centre of the image in pixels
     */
    Camera::Camera( const float focal_x, const float focal_y, const float centre_x, const float centre_y )  {
        m_k << -focal_x, 0.0f, centre_x, 0.0f, -focal_y, centre_y, 0.0f, 0.0f, 1.0f;
        m_k_inverse = m_k.inverse();
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
        m_k_inverse = m_k.inverse();
    }
    
    /**
     * Construct a camera given the image dimensions and field of view
     * @param image_width The width of the image in pixels
     * @param image_height The height of the image in pixels
     * @param fov_x The horizontal field of view in mm
     * @param fov_y The vertical field of view in mm
     */
    Camera::Camera( const int image_width, const int image_height, const float fov_x, const float fov_y ) {
        float focal_x = -image_width / ( 2 * std::tanf(fov_x / 2.0f ) );
        float focal_y = -image_height / ( 2 * std::tanf(fov_y / 2.0f ) );
        m_k << -focal_x, 0.0f, (image_width / 2.0f), 0.0f, -focal_y, (image_height / 2.0f), 0.0f, 0.0f, 0.0f, 1.0f;
        m_k_inverse = m_k.inverse();
    }
    
    /**
     * Convert from image plane coordinates to camera space coordinates
     * @param image_coordinate The 2D coordinate in the image space
     * @param camera_coordinate The 2D coordinate in camera image plane
     */
    void Camera::image_to_camera( const Eigen::Vector2i & image_coordinate, Eigen::Vector2f & camera_coordinate ) const {
        using namespace Eigen;
        
        image_to_camera(image_coordinate.x(), image_coordinate.y(), camera_coordinate );
    }
    
    /**
     * Convert from image plane coordinates to camera space coordinates
     * @param image_x The x coordinate in the image space
     * @param image_y The y coordinate in the image space
     * @param camera_coordinate The 2D coordinate in camera image plane
     */
    void Camera::image_to_camera( const int x, const int y, Eigen::Vector2f & camera_coordinate ) const {
        using namespace Eigen;
        
        // Multiply the (homgeous) point by the intrinsic matrix
        Vector3f homogenous_coordinate{ x, y, 1.0f };
        Vector3f homogenous_camera_coordinate = m_k_inverse * homogenous_coordinate;
        camera_coordinate.x() = homogenous_camera_coordinate[0] / homogenous_camera_coordinate[2];
        camera_coordinate.y() = homogenous_camera_coordinate[1] / homogenous_camera_coordinate[2];
    }
    
    /**
     * Convert from a depth image to 3D camera space coordinates
     * @param depth_image A width x height array of uint16_t depth values
     * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
     * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
     */
    void Camera::depth_image_to_vertices_and_normals( const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> & depth_image, Eigen::ArrayBase<Eigen::Vector3f> & vertices, Eigen::ArrayBase<Eigen::Vector3f> & normals ) const {
        uint16_t width = depth_image.cols();
        uint16_t height = depth_image.rows();
        
        for( uint16_t y=0; y<height; y++ ) {
            for( uint16_t x=0; x<width; x++ ) {
                // Back project the point into camera 3D space using D(x,y) * Kinv * (x,y,1)T
                imagee_to_camera(Vector2i{ x, y }
            }
        }
    }
}