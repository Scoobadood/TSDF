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
     * Predefined bad vertex
     */
    Eigen::Vector3f BAD_VERTEX{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    
    /**
     * Common construction code
     */
    void Camera::init( ) {
        m_k_inverse = m_k.inverse();
        m_pose << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
    }
    
    /**
     * Construct a camera with the given intrinsic parameters
     * @param focal_x The focal length in the x direction in pixels
     * @param focal_y The focal length in the y direction in pixels
     * @param centre_x The horizontal centre of the image in pixels
     * @param centre_y The vertical centre of the image in pixels
     */
    Camera::Camera( const float focal_x, const float focal_y, const float centre_x, const float centre_y )  {
        m_k << -focal_x, 0.0f, centre_x, 0.0f, -focal_y, centre_y, 0.0f, 0.0f, 1.0f;
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
        float focal_x = -image_width / ( 2 * std::tanf(fov_x / 2.0f ) );
        float focal_y = -image_height / ( 2 * std::tanf(fov_y / 2.0f ) );
        m_k << -focal_x, 0.0f, (image_width / 2.0f), 0.0f, -focal_y, (image_height / 2.0f), 0.0f, 0.0f, 1.0f;
         
         init() ;
    }
    
#pragma mark - Pose
    /**
     * @return the pose of the camera as a 4x4 matrix
     */
    const Eigen::Matrix4f & Camera::pose( ) const {
        return m_pose;
    }

    /**
     * @param The new pose of the camera as a 4x4 matrix
     */
    void Camera::set_pose( const Eigen::Matrix4f & pose ) {
        m_pose = pose;
    }

    
    
#pragma mark - Camera coordinate methods
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
     * Convert from camera coordinates to image coordinates
     * @param camera_x The x coordinate in the camera plane
     * @param camera_y The y coordinate in the camera plane
     * @param image_coordinate The 2D coordinate in camera's image
     */
    void Camera::camera_to_image_plane( const float x, const float y, Eigen::Vector2i & image_coordinate ) const {
        using namespace Eigen;
        
        camera_to_image_plane(Vector3f{ x, y, 1.0f }, image_coordinate );
    }
    
    /**
     * Convert from camera coordinates to image coordinates
     * @param camera_coordinates The point coorinates in camera plane homegenous
     * @param image_coordinate The 2D coordinate in camera's image
     */
    void Camera::camera_to_image_plane( const Eigen::Vector3f camera_coordinates, Eigen::Vector2i & image_coordinate ) const {
        using namespace Eigen;
        
        Vector3f image_h = m_k * camera_coordinates;
        
        image_coordinate.x() = std::roundf(image_h[0] / image_h[2]);
        image_coordinate.y() = std::roundf(image_h[1] / image_h[2]);
    }
    
    /**
     * Convert the global coordinates into camera space
     * Multiply by pose.inverse()
     * @param world_coordinate The 3D point in world space
     * @param camera_coordinate The 3D pointin camera space
     */
    void Camera::world_to_camera( const Eigen::Vector3f & world_coordinate, Eigen::Vector3f & camera_coordinate ) const {
        using namespace Eigen;
        
        Vector4f world_h{ world_coordinate.x(), world_coordinate.y(), world_coordinate.z(), 1.0f};
        Vector4f cam_h = m_pose.inverse() * world_h;
        
        camera_coordinate = cam_h.block(0, 0, 3, 1) / cam_h.w();
    }
    
    /**
     * Convert the global coordinates into camera space
     * Multiply by pose.inverse()
     * @param world_coordinate The 3D point in world space
     * @param camera_coordinate The 3D pointin camera space
     */
    void Camera::world_to_image( const Eigen::Vector3f & world_coordinate, Eigen::Vector2i & image_coordinate ) const {
        using namespace Eigen;
        
        Vector3f cam_coordinate;
        world_to_camera(world_coordinate, cam_coordinate);
        camera_to_image_plane(cam_coordinate, image_coordinate);
    }
    
#pragma mark - Depth map methods
    /**
     * Convert from a depth image to 3D camera space coordinates
     * @param depth_image A width x height array of uint16_t depth values
     * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
     * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
     */
    void Camera::depth_image_to_vertices_and_normals(const uint16_t * depth_image,const uint32_t width,const uint32_t height,  std::deque<Eigen::Vector3f> & vertices,  std::deque<Eigen::Vector3f> & normals ) const {
        using namespace Eigen;
        
        // Run from bottom right corner so we can create normal sin the same pass
        int32_t src_idx = (width * height) - 1;
        for( int16_t y=height-1; y>=0; y-- ) {
            for( int16_t x=width-1; x >= 0; x-- ) {
                
                // Vertex and normal for this pixel
                Vector3f vertex;
                Vector3f normal{ 0, 0, 0 };
                
                // If this is a valid depth
                uint16_t depth = depth_image[src_idx];
                if( depth != 0 ) {
                    
                    // Back project the point into camera 3D space using D(x,y) * Kinv * (x,y,1)T
                    Vector2f cam_point;
                    image_to_camera( x, y, cam_point );
                    
                    // Create the actual vertex
                    vertex = Vector3f{ cam_point.x() * depth, cam_point.y() * depth, depth };
                    
                    // Compute normal as v(y,x+1)-v(y,x) cross v(y+1, x)-v(y, x )
                    if( (y < height - 1 ) && ( x < width - 1 ) ) {
                        // We have adjacent vertex to right and below that we can extract
                        // Vector[0] is the element to the right of this one
                        // Vector[width] is the element below
                        Vector3f right_neighbour = vertices[0];
                        Vector3f below_neighbour = vertices[width];
                        
                        // If they are both not BAD
                        if( ( right_neighbour != BAD_VERTEX) && ( below_neighbour != BAD_VERTEX ) ){
                            right_neighbour -= vertex;
                            below_neighbour -= vertex;
                            
                            // Compute cros product for normal
                            normal = right_neighbour.cross( below_neighbour ).normalized();
                        }
                    }
                } else {
                    vertex = BAD_VERTEX;
                }
                
                // Store
                vertices.push_front( vertex );
                normals.push_front( normal );
                
                src_idx--;
            }
        }
    }
    
    /**
     * Convert from a depth image to 3D camera space coordinates
     * @param depth_image A width x height array of uint16_t depth values
     * @param vertices A width x height array of Vector3f representing the vertices in the depth image in camera space
     * @param normals A width x height array of Vector3f representing the vertices in the depth image in camera space
     */
    void Camera::depth_image_to_vertices_and_normals(
                                                     const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> & depth_image,
                                                     std::deque<Eigen::Vector3f> & vertices,
                                                     std::deque<Eigen::Vector3f> & normals ) const {
        using namespace Eigen;
        
        uint16_t width = depth_image.cols();
        uint16_t height = depth_image.rows();
        
        // Run from bottom right corner so we can create normal sin the same pass
        for( int16_t y=height-1; y>=0; y-- ) {
            for( int16_t x=width; x >= 0; x-- ) {
                
                // Vertex and normal for this pixel
                Vector3f vertex;
                Vector3f normal{ 0, 0, 0 };
                
                // If this is a valid depth
                uint16_t depth = depth_image( y, x );
                if( depth != 0 ) {
                    
                    // Back project the point into camera 3D space using D(x,y) * Kinv * (x,y,1)T
                    Vector2f cam_point;
                    image_to_camera( x, y, cam_point );
                    vertex = Vector3f{ cam_point.x() * depth, cam_point.y() * depth, depth };
                    
                    // Compute normal as v(y,x+1)-v(y,x) cross v(y+1, x)-v(y, x )
                    if( (y < height - 1 ) && ( x < width - 1 ) ) {
                        // We have adjacent vertex to right and below that we can extract
                        // Vector[0] is the element to the right of this one
                        // Vector[width] is the element below
                        Vector3f right_neighbour = vertices[0];
                        Vector3f below_neighbour = vertices[width];
                        
                        // If they are both not BAD
                        if( ( right_neighbour != BAD_VERTEX) && ( below_neighbour != BAD_VERTEX ) ){
                            right_neighbour -= vertex;
                            below_neighbour -= vertex;
                            
                            // Compute cros product for normal
                            normal = right_neighbour.cross( below_neighbour );
                        }
                    }
                } else {
                    vertex = BAD_VERTEX;
                }
                
                // Store
                vertices.push_front( vertex );
                normals.push_front( normal );
            }
        }
    }
}
