//
//  TestHelpers.hpp
//  KinFu
//
//  Created by Dave on 16/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef TestHelpers_h
#define TestHelpers_h

#include "PngUtilities.hpp"
#include "TSDFVolume.hpp"


#pragma mark - helpers

phd::TSDFVolume construct_volume( int vx, int vy, int vz, float px, float py, float pz, float & vw, float & vh, float & vd);
void create_sphere_in_TSDF( phd::TSDFVolume & volume, float radius );
void create_wall_in_TSDF( phd::TSDFVolume & volume, float depth );
void save_normals_as_png( std::string filename, uint16_t width, uint16_t height, Eigen::Vector3f * normals );
void save_normals_as_colour_png( std::string filename, uint16_t width, uint16_t height, Eigen::Vector3f * normals );
void save_rendered_scene_as_png(std::string filename, uint16_t width, uint16_t height, Eigen::Vector3f * vertices, Eigen::Vector3f * normals, const Eigen::Vector3f & camera_position, const Eigen::Vector3f & light_source);

#endif /* TestHelpers_h */
