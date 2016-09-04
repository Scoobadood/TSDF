#include "TSDFVolume.hpp"
#include "CPU/CPUTSDFVolume.hpp"
#include "GPU/GPUTSDFVolume.hpp"


/**
 * Factory method to return a CPU or GPU based volume
 * @param volume_x X dimension in voxels
 * @param volume_y Y dimension in voxels
 * @param volume_z Z dimension in voxels
 * @param psize_x Physical size in X dimension in mm
 * @param psize_y Physical size in Y dimension in mm
 * @param psize_z Physical size in Z dimension in mm
 */
TSDFVolume * TSDFVolume::make_volume( TSDFVolume::volume_type type,
                                const Eigen::Vector3i& voxel_size,
                                const Eigen::Vector3f& physical_size ) {
    using namespace Eigen;
    TSDFVolume *volume = NULL;

    switch (type) {
    case TSDFVolume::CPU:
        volume = new CPUTSDFVolume { voxel_size, physical_size};
        break;

    case TSDFVolume::GPU:
        volume = new GPUTSDFVolume { voxel_size, physical_size};
        break;

    }

    return volume;
}

/**
 * Factory method to return a CPU or GPU based volume
 * @param volume_x X dimension in voxels
 * @param volume_y Y dimension in voxels
 * @param volume_z Z dimension in voxels
 * @param psize_x Physical size in X dimension in mm
 * @param psize_y Physical size in Y dimension in mm
 * @param psize_z Physical size in Z dimension in mm
 */
TSDFVolume *TSDFVolume::make_volume( TSDFVolume::volume_type type, uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z ) {
    using namespace Eigen;

    return TSDFVolume::make_volume( type, Vector3i{ volume_x, volume_y, volume_z}, Vector3f{ psize_x, psize_y, psize_z});
}
