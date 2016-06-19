#include "TSDFVolume.hpp"
#include "CPU/CPUTSDFVolume.hpp"

namespace phd {

/**
 * Factory method to return a CPU or GPU based volume
 */
TSDFVolume *TSDFVolume::make_volume( TSDFVolume::volume_type type ) {
    TSDFVolume *volume = NULL;

    switch (type) {
    case TSDFVolume::CPU:
        volume = new CPUTSDFVolume {};

    case TSDFVolume::GPU:
        return NULL;
    }

    return volume;
}
}
