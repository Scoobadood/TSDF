//
//  Constants.hpp
//  WarpField
//
//  Created by Dave on 8/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef Constants_hpp
#define Constants_hpp

#include <iostream>
#include <gtest/gtest.h>

#include "Eigen/Core"
#include "math.h"
#include "Quaternion.hpp"

extern const double EPS;
extern const double HALF_ROOT_3;
extern const double DEGREES_30;
extern const Eigen::Vector3d X_AXIS;
extern const Eigen::Vector3d Y_AXIS;
extern const Eigen::Vector3d Z_AXIS;
extern const Eigen::Vector3d IDENTITY_TRANSLATION;
extern const phd::Quaternion IDENTITY_ROTATION;
extern const phd::Quaternion NULL_QUATERNION;

#endif /* Constants_hpp */
