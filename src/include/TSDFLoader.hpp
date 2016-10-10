#ifndef TSDFLOADER_H
#define TSDFLOADER_H

#include "TSDFVolume.hpp"

class TSDFLoader {
public:
    TSDFLoader( TSDFVolume * volume );
    virtual ~TSDFLoader();
    void process_line( const std::string & line );
    bool load_from_file( const std::string & file_name );
protected:
    void process_voxel_size_line( const std::string & line );
    void process_physical_size_line( const std::string & line );
    void process_distance_line( const std::string & line );
    void process_weight_line( const std::string & line );

private:
    TSDFVolume  *m_volume;
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

#endif // TSDFLOADER_H
