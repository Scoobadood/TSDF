//
//  Constants.cpp
//  WarpField
//
//  Created by Dave on 8/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Constants.hpp"

const double EPS = 1e-6;
const double HALF_ROOT_3 = sqrt( 3.0 ) / 2.0;
const double DEGREES_30 = M_PI / 6.0;
const Eigen::Vector3d X_AXIS{ 1.0, 0.0, 0.0};
const Eigen::Vector3d Y_AXIS{ 0.0, 1.0, 0.0};
const Eigen::Vector3d Z_AXIS{ 0.0, 0.0, 1.0};
const Eigen::Vector3d IDENTITY_TRANSLATION{0.0, 0.0, 0.0};
const Quaternion IDENTITY_ROTATION{1.0, 0.0, 0.0, 0.0};
const Quaternion NULL_QUATERNION{0.0, 0.0, 0.0, 0.0};


