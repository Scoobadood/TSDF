//
//  Raycaster.hpp
//  KinFu
//
//  Created by Dave on 2/06/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef Raycaster_hpp
#define Raycaster_hpp

#include <Eigen/Core>

#include "TSDFVolume.hpp"
#include "Camera.hpp"

    class Raycaster {
    public:
        Raycaster( int width=640, int height=480) {
            m_width = width;
            m_height = height;
        }

        /**
         * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
         * @param volume The volume to cast
         * @param camera The camera
         * @param vertices The vertices discovered
         * @param normals The normals
         */
        virtual void raycast( const TSDFVolume & volume, const Camera & camera, Eigen::Matrix<float, 3, Eigen::Dynamic> &  vertices, Eigen::Matrix<float, 3, Eigen::Dynamic> &  normals ) const = 0;


    protected:

        uint16_t    m_width;
        uint16_t    m_height;

    };
#endif /* Raycaster_hpp */
