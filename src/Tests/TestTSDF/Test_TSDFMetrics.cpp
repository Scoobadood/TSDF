//
//  TestTSDFVoxels.cpp
//  KinFu
//
//  Created by Dave on 15/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//


#include <stdio.h>
#include <gtest/gtest.h>
#include <cfloat>

#include "../KinFu/TSDFVolume.hpp"

#include "TestHelpers.hpp"

const float EPS = 1e-6;

#pragma mark - Construction dimensions
TEST( TSDF_CPU_Metrics, givenValidDimensionsWhenConstructingThenResolutionIsComputedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 9, 12, 15 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );
    Eigen::Vector3f voxel_size = volume->voxel_size();

    EXPECT_NEAR( voxel_size.x(), 1000.0f / 3.0f, EPS );
    EXPECT_NEAR( voxel_size.y(), 250.0f, EPS );
    EXPECT_NEAR( voxel_size.z(), 200.0f, EPS );

    delete volume;
}

TEST( TSDF_CPU_Metrics, givenDefaultOffsetWhenGettingCentreOfVolumeThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    CPUTSDFVolume * volume = (CPUTSDFVolume *)TSDFVolume::make_volume(TSDFVolume::CPU, size );

    Eigen::Vector3f centre = volume->centre_of_volume();
    EXPECT_NEAR( centre[0], 1500.0f, EPS );
    EXPECT_NEAR( centre[1], 1500.0f, EPS );
    EXPECT_NEAR( centre[2], 1500.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenPositiveOffsetWhenGettingCentreOfVolumeThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    volume->offset(100, 200, 300);

    Eigen::Vector3f centre = volume->centre_of_volume();
    EXPECT_NEAR( centre[0], 1600.0f, EPS );
    EXPECT_NEAR( centre[1], 1700.0f, EPS );
    EXPECT_NEAR( centre[2], 1800.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenNegativeOffsetWhenGettingCentreOfVolumeThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    volume->offset(-100, -200, -300);

    Eigen::Vector3f centre = volume->centre_of_volume();
    EXPECT_NEAR( centre[0], 1400.0f, EPS );
    EXPECT_NEAR( centre[1], 1300.0f, EPS );
    EXPECT_NEAR( centre[2], 1200.0f, EPS );
}


TEST( TSDF_CPU_Metrics, givenPositiveOffsetWithSpecificSizeWhenGettingCentreOfVolumeThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size, Eigen::Vector3f{100, 50, 20} );

    volume->offset(100, 200, 300);

    Eigen::Vector3f centre = volume->centre_of_volume();
    EXPECT_NEAR( centre[0], 150.0f, EPS );
    EXPECT_NEAR( centre[1], 225.0f, EPS );
    EXPECT_NEAR( centre[2], 310.0f, EPS );
}

#pragma mark - voxel and volume centre

TEST( TSDF_CPU_Metrics, givenDefaultOffsetWhenGettingCentreOfVoxelAt_0_0_0ThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(0, 0, 0);
    EXPECT_NEAR( centre[0], 500.0f, EPS );
    EXPECT_NEAR( centre[1], 375.0f, EPS );
    EXPECT_NEAR( centre[2], 300.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenDefaultOffsetWhenGettingCentreOfVoxelAt_MaxX_0_0ThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(2, 0, 0);
    EXPECT_NEAR( centre[0], 2500.0f, EPS );
    EXPECT_NEAR( centre[1], 375.0f, EPS );
    EXPECT_NEAR( centre[2], 300.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenDefaultOffsetWhenGettingCentreOfVoxelAt_0_MaxY_0ThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(0, 3, 0);
    EXPECT_NEAR( centre[0], 500.0f, EPS );
    EXPECT_NEAR( centre[1], 2625.0f, EPS );
    EXPECT_NEAR( centre[2], 300.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenDefaultOffsetWhenGettingCentreOfVoxelAt_0_0_MaxZThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(0, 0, 4);
    EXPECT_NEAR( centre[0], 500.0f, EPS );
    EXPECT_NEAR( centre[1], 375.0f, EPS );
    EXPECT_NEAR( centre[2], 2700.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenOffsetWhenGettingCentreOfVoxelAt_0_0_0ThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };
    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    volume->offset(-1500.0f, -2000.0f, -2500.0f );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(0, 0, 0);
    EXPECT_NEAR( centre[0], -1000.0f, EPS );
    EXPECT_NEAR( centre[1], -1625.0f, EPS );
    EXPECT_NEAR( centre[2], -2200.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenOffsetWhenGettingCentreOfVoxelAt_MaxX_0_0ThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };
    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );
    volume->offset(-1500.0f, -2000.0f, -2500.0f );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(2, 0, 0);
    EXPECT_NEAR( centre[0], 1000.0f, EPS );
    EXPECT_NEAR( centre[1], -1625.0f, EPS );
    EXPECT_NEAR( centre[2], -2200.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenOffsetWhenGettingCentreOfVoxelAt_0_MaxY_0ThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };
    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );
    volume->offset(-1500.0f, -2000.0f, -2500.0f );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(0, 3, 0);
    EXPECT_NEAR( centre[0], -1000.0f, EPS );
    EXPECT_NEAR( centre[1], 625.0f, EPS );
    EXPECT_NEAR( centre[2], -2200.0f, EPS );
}

TEST( TSDF_CPU_Metrics, givenOffsetWhenGettingCentreOfVoxelAt_0_0_MaxZThenCentreIsReturnedAccurately ) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 4, 5 };
    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );
    volume->offset(-1500.0f, -2000.0f, -2500.0f );

    Eigen::Vector3f centre = volume->centre_of_voxel_at(0, 0, 4);
    EXPECT_NEAR( centre[0], -1000.0f, EPS );
    EXPECT_NEAR( centre[1], -1625.0f, EPS );
    EXPECT_NEAR( centre[2], 200.0f, EPS );
}

#pragma volume centre

TEST( TSDF_CPU_Metrics,  givenDefaultVolumeWhenFindingCentreOfVolumeCentreIsCorrect) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 3, 3 };
    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size );

    Eigen::Vector3f centre = volume->centre_of_volume();

    EXPECT_NEAR( centre.x(), 1500.0f , EPS );
    EXPECT_NEAR( centre.y(),  1500.0f, EPS );
    EXPECT_NEAR( centre.z(),  1500.0f, EPS );
}

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingCentreOfVolumeCentreIsCorrect) {
    using namespace phd;

    Eigen::Vector3i size{ 3, 3, 3 };
    Eigen::Vector3f physical_size{ 1000.0f, 4000.0f, 900.0f };
    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, size, physical_size );

    Eigen::Vector3f centre = volume->centre_of_volume();

    EXPECT_NEAR( centre.x(),  500.0f , EPS );
    EXPECT_NEAR( centre.y(), 2000.0f, EPS );
    EXPECT_NEAR( centre.z(),  450.0f, EPS );
}

#pragma mark - Voxel boundaries

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingVoxelBoundssAt_0_0_0_ThenBoundsAreCorrect) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 3, 2 }, Eigen::Vector3f{1000.0f, 2000.0f, 3000.0f} );
    Eigen::Vector3f vsize = volume->voxel_size();

    Eigen::Vector3f actual_lower_bounds;
    Eigen::Vector3f actual_upper_bounds;
    volume->voxel_bounds( Eigen::Vector3i{ 0, 0, 0 }, actual_lower_bounds, actual_upper_bounds);

    EXPECT_NEAR( actual_lower_bounds.x(),  0.0f , EPS );
    EXPECT_NEAR( actual_lower_bounds.y(),  0.0f , EPS );
    EXPECT_NEAR( actual_lower_bounds.z(),  0.0f , EPS );

    EXPECT_NEAR( actual_upper_bounds.x(),  vsize[0], EPS );
    EXPECT_NEAR( actual_upper_bounds.y(),  vsize[1] , EPS );
    EXPECT_NEAR( actual_upper_bounds.z(),  vsize[2] , EPS );
}

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingVoxelBoundssAt_MaxX_0_0_ThenBoundsAreCorrect) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 3, 2 }, Eigen::Vector3f{1000.0f, 2000.0f, 3000.0f} );
    Eigen::Vector3f vsize = volume->voxel_size();

    Eigen::Vector3f actual_lower_bounds;
    Eigen::Vector3f actual_upper_bounds;
    volume->voxel_bounds( Eigen::Vector3i{ 3, 0, 0 }, actual_lower_bounds, actual_upper_bounds);

    EXPECT_NEAR( actual_lower_bounds.x(),  3 * vsize[0] , EPS );
    EXPECT_NEAR( actual_lower_bounds.y(),  0.0f , EPS );
    EXPECT_NEAR( actual_lower_bounds.z(),  0.0f , EPS );

    EXPECT_NEAR( actual_upper_bounds.x(),  4 * vsize[0] , EPS );
    EXPECT_NEAR( actual_upper_bounds.y(),  vsize[1] , EPS );
    EXPECT_NEAR( actual_upper_bounds.z(),  vsize[2] , EPS );
}

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingVoxelBoundssAt_0_MaxY_0_ThenBoundsAreCorrect) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 3, 2 }, Eigen::Vector3f{1000.0f, 2000.0f, 3000.0f} );
    Eigen::Vector3f vsize = volume->voxel_size();

    Eigen::Vector3f actual_lower_bounds;
    Eigen::Vector3f actual_upper_bounds;
    volume->voxel_bounds( Eigen::Vector3i{ 0, 2, 0 }, actual_lower_bounds, actual_upper_bounds);

    EXPECT_NEAR( actual_lower_bounds.x(),  0.0f , EPS );
    EXPECT_NEAR( actual_lower_bounds.y(),  2 * vsize[1] , EPS );
    EXPECT_NEAR( actual_lower_bounds.z(),  0.0f , EPS );

    EXPECT_NEAR( actual_upper_bounds.x(),  vsize[0] , EPS );
    EXPECT_NEAR( actual_upper_bounds.y(),  3 * vsize[1] , EPS );
    EXPECT_NEAR( actual_upper_bounds.z(),  vsize[2] , EPS );
}

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingVoxelBoundssAt_0_0_MaxZ_ThenBoundsAreCorrect) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 3, 2 }, Eigen::Vector3f{1000.0f, 2000.0f, 3000.0f} );
    Eigen::Vector3f vsize = volume->voxel_size();

    Eigen::Vector3f actual_lower_bounds;
    Eigen::Vector3f actual_upper_bounds;
    volume.voxel_bounds( Eigen::Vector3i{ 0, 0, 1 }, actual_lower_bounds, actual_upper_bounds);

    EXPECT_NEAR( actual_lower_bounds.x(),  0.0f , EPS );
    EXPECT_NEAR( actual_lower_bounds.y(),  0.0f , EPS );
    EXPECT_NEAR( actual_lower_bounds.z(),  vsize[2] , EPS );

    EXPECT_NEAR( actual_upper_bounds.x(),  vsize[0] , EPS );
    EXPECT_NEAR( actual_upper_bounds.y(),  vsize[1] , EPS );
    EXPECT_NEAR( actual_upper_bounds.z(),  2 * vsize[2] , EPS );
}

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingVoxelBoundssAt_MaxX_MaxY_MaxZ_ThenBoundsAreCorrect) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 3, 2 }, Eigen::Vector3f{1000.0f, 2000.0f, 3000.0f} );
    Eigen::Vector3f vsize = volume->voxel_size();

    Eigen::Vector3f actual_lower_bounds;
    Eigen::Vector3f actual_upper_bounds;
    volume.voxel_bounds( Eigen::Vector3i{ 3, 2, 1 }, actual_lower_bounds, actual_upper_bounds);

    EXPECT_NEAR( actual_lower_bounds.x(),  3 * vsize[0] , EPS );
    EXPECT_NEAR( actual_lower_bounds.y(),  2 * vsize[1] , EPS );
    EXPECT_NEAR( actual_lower_bounds.z(),  vsize[2] , EPS );

    EXPECT_NEAR( actual_upper_bounds.x(),  4 * vsize[0] , EPS );
    EXPECT_NEAR( actual_upper_bounds.y(),  3 * vsize[1] , EPS );
    EXPECT_NEAR( actual_upper_bounds.z(),  2 * vsize[2] , EPS );
}

TEST( TSDF_CPU_Metrics,  givenSizedVolumeWhenFindingVoxelBoundssAtInvalidVoxelThenExceptionThrown) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 3, 2 }, Eigen::Vector3f{1000.0f, 2000.0f, 3000.0f} );
    Eigen::Vector3f vsize = volume->voxel_size();

    Eigen::Vector3f actual_lower_bounds;
    Eigen::Vector3f actual_upper_bounds;
    volume.voxel_bounds( Eigen::Vector3i{ 3, 2, 1 }, actual_lower_bounds, actual_upper_bounds);

    EXPECT_THROW( volume.voxel_bounds( Eigen::Vector3i{ -1,  1,  1 }, actual_lower_bounds, actual_upper_bounds), std::invalid_argument );
    EXPECT_THROW( volume.voxel_bounds( Eigen::Vector3i{  1, -1,  1 }, actual_lower_bounds, actual_upper_bounds), std::invalid_argument );
    EXPECT_THROW( volume.voxel_bounds( Eigen::Vector3i{  1,  1, -1 }, actual_lower_bounds, actual_upper_bounds), std::invalid_argument );
    EXPECT_THROW( volume.voxel_bounds( Eigen::Vector3i{  4,  1,  1 }, actual_lower_bounds, actual_upper_bounds), std::invalid_argument );
    EXPECT_THROW( volume.voxel_bounds( Eigen::Vector3i{  1,  3,  1 }, actual_lower_bounds, actual_upper_bounds), std::invalid_argument );
    EXPECT_THROW( volume.voxel_bounds( Eigen::Vector3i{  1,  1,  2 }, actual_lower_bounds, actual_upper_bounds), std::invalid_argument );
}

#pragma mark - Voxel containment
TEST( TSDF_CPU_Metrics,  givenVolumeWhenFindingValidVoxelThenReturnsTrue) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 5, 6 } );

    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 0, 0, 0} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 0, 0, 5} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 0, 4, 0} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 0, 4, 5} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 3, 0, 0} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 3, 0, 5} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 3, 4, 0} ) );
    EXPECT_TRUE( volume->contains_voxel( Eigen::Vector3i{ 3, 4, 5} ) );
}

TEST( TSDF_CPU_Metrics,  givenVolumeWhenFindingInvalidVoxelThenReturnsFalse) {
    using namespace phd;

    TSDFVolume * volume = TSDFVolume::make_volume(TSDFVolume::CPU, Eigen::Vector3i{ 4, 5, 6 } );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ 0,  0, -1} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ 0,  0,  6} ) );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ 0, -1,  0} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ 0,  5,  0} ) );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ 0, -1, -1} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ 0,  5,  6} ) );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ -1,  0,  0} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{  4,  0,  0} ) );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ -1,  0, -1} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{  4,  0,  6} ) );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ -1, -1,  0} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{  4,  5,  0} ) );

    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{ -1, -1, -1} ) );
    EXPECT_FALSE( volume->contains_voxel( Eigen::Vector3i{  4,  5,  6} ) );
}




