//
//  TestHelpers.hpp
//  KinFu
//
//  Created by Dave on 16/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef TestHelpers_h
#define TestHelpers_h

#include "../../include/TSDFVolume.hpp"
#include "../../include/Camera.hpp"
#include "../../include/PngWrapper.hpp"


#include <Eigen/Dense>

#pragma mark - helpers


void create_sphere_in_TSDF( TSDFVolume & volume, float radius );
void create_wall_in_TSDF( TSDFVolume & volume, float depth );

uint16_t * make_sphere_depth_map( uint16_t width, uint16_t height, uint16_t radius, uint16_t min_depth, uint16_t max_depth ) ;
uint16_t * make_wall_depth_map( uint16_t width, uint16_t height, uint16_t max_depth, uint16_t min_depth, uint16_t wall_depth );


void save_depth_map( std::string file_name, uint16_t width, uint16_t height, uint16_t * pixels);

Eigen::Matrix4f make_y_axis_rotation( float theta, Eigen::Vector3f pos );
Eigen::Matrix4f make_x_axis_rotation( float theta, Eigen::Vector3f pos );
Eigen::Matrix4f make_z_axis_rotation( float theta, Eigen::Vector3f pos );

void helper_move_look( float vars[7], Eigen::Vector3f & move_to, Eigen::Vector3f & look_at );


#endif /* TestHelpers_h */
