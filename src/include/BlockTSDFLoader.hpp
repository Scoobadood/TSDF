#ifndef TSDF_BLOCK_LOADER_H
#define TSDF_BLOCK_LOADER_H

#include "TSDFVolume.hpp"
#include "FileUtilities.hpp"




class TSDFVolume;

class BlockTSDFLoader {
public:
    BlockTSDFLoader( );
    ~BlockTSDFLoader();


    bool load_from_file( const std::string & file_name );
    /**
     * Process the line
     */
    void process_line( const std::string & line );

    TSDFVolume * to_tsdf( TSDFVolume::volume_type type ) const;

protected:
    void process_voxel_size_line( const std::string & line );
    void process_physical_size_line( const std::string & line );
    void process_distance_line( const std::string & line );
    void process_weight_line( const std::string & line );

private:
    float       * m_distance_data;
    float       * m_weight_data;

    uint16_t    m_size_x, m_size_y, m_size_z;
    float       m_psize_x, m_psize_y, m_psize_z;
    uint16_t    m_x, m_y;
    enum {
        expect_voxel_size,
        expect_physical_size,
        expect_weights,
        expect_distances,
        done,
        data_ignored
    } m_state;
};

#endif