//
//  TestCamera.cpp
//  KinFu
//
//  Created by Dave on 19/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <gtest/gtest.h>
#include "Camera.hpp"

#pragma mark - Construction

TEST( TSDF_Construction, givenNegativeXDimensionWhenConstructingThenThrowsException ) {
    using namespace phd;
    
    Eigen::Vector3i size{ -10, 11, 12 };
    
    EXPECT_THROW(TSDFVolume volume{ size }, std::invalid_argument);
}

