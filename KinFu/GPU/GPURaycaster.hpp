//
//  Raycaster.hpp
//  KinFu
//
//  Created by Dave on 2/06/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef GPURaycaster_hpp
#define GPURaycaster_hpp

#include <Eigen/Core>
#include "../Raycaster.hpp"


namespace phd {
    class GPURaycaster : public Raycaster {
    public:
        GPURaycaster( int width=640, int height=480) : Raycaster{ width, height } {} ;

        /**
         * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
         * @param volume The volume to cast
         * @param camera The camera
         * @param vertices The vertices discovered
         * @param normals The normals
         */
        virtual void raycast( const TSDFVolume & volume, const Camera & camera,
                     Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
                     Eigen::Matrix<float, 3, Eigen::Dynamic> & normals ) const;
    };
}
#endif /* GPURaycaster_hpp */
