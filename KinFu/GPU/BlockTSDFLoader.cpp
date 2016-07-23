#include "BlockTSDFLoader.hpp"

namespace phd {
BlockTSDFLoader::BlockTSDFLoader( ) {
    // Set up state
    m_state = expect_voxel_size;
    m_distance_data = nullptr;
    m_weight_data = nullptr;
}

BlockTSDFLoader::~BlockTSDFLoader() {
    if ( m_distance_data != nullptr ) {
        delete [] m_distance_data;
        m_distance_data = nullptr;
    }
    if ( m_weight_data != nullptr ) {
        delete [] m_weight_data;
        m_weight_data = nullptr;
    }
}

/**
 * Read the voxel size
 */
void BlockTSDFLoader::process_voxel_size_line( const std::string & line ) {
    std::stringstream iss(line);
    std::string voxel_size_prefix;
    std::getline(iss, voxel_size_prefix, '=');
    iss >> m_size_x >> m_size_y >> m_size_z;

    // Allocate storage
    m_distance_data = new float[ m_size_x * m_size_y * m_size_z ];
    if ( m_distance_data != nullptr ) {
        m_weight_data = new float[ m_size_x * m_size_y * m_size_z ];
        if ( m_weight_data == nullptr) {
            delete [] m_distance_data;
            throw std::bad_alloc();
        }
    } else {
        throw std::bad_alloc();
    }

    m_state = expect_physical_size;
}
/**
 * Read the physical size
 */
void BlockTSDFLoader::process_physical_size_line( const std::string & line ) {
    std::stringstream iss(line);
    std::string physical_size_prefix;
    std::getline(iss, physical_size_prefix, '=');
    iss >> m_psize_x >> m_psize_y >> m_psize_z;

    m_x = 0;
    m_y = 0;

    m_state = expect_distances;
}
/**
 * Read a line of distances
 */
void BlockTSDFLoader::process_distance_line( const std::string & line ) {
    std::stringstream iss(line);
    for ( uint16_t z = 0; z < m_size_z; z++ ) {
        float distance;
        iss >> distance;

        size_t idx = ((m_size_x * m_size_y) * z)  + (m_size_x * m_y) + m_x;
        m_distance_data[ idx ] = distance;
    }
    m_state = expect_weights;
}
/**
 * Read a line of weights
 */
void BlockTSDFLoader::process_weight_line( const std::string & line ) {
    std::stringstream iss(line);
    for ( uint16_t z = 0; z < m_size_z; z++ ) {
        float weight;
        iss >> weight;

        size_t idx = ((m_size_x * m_size_y) * z)  + (m_size_x * m_y) + m_x;
        m_weight_data[ idx ] = weight;
    }

    m_state = expect_distances;
    if ( ++m_x == m_size_x ) {
        m_x = 0;
        if ( ++m_y == m_size_y ) {
            m_state = done;
        }
    }
}

bool BlockTSDFLoader::load_from_file( const std::string & file_name ) {
    std::function<void( const std::string & )> f = std::bind(&phd::BlockTSDFLoader::process_line, this, std::placeholders::_1 );
    process_file_by_lines( file_name, f);

    // Assert that we have the correct state
    return ( m_state == done );
}
/**
   * Process the line
   */
void BlockTSDFLoader::process_line( const std::string & line ) {
    // Fail empty lines
    if ( line.size() == 0 ) return;

    // Ignore comments
    if ( line[0] == '#') return;

    // If I've done reading ignore this line
    if ( m_state == data_ignored ) return;


    // Handle the remaininder depending on state
    switch ( m_state ) {
    case expect_voxel_size:
        process_voxel_size_line(line);
        break;

    case expect_physical_size:
        process_physical_size_line(line);
        break;

    case expect_weights:
        process_weight_line( line );
        break;

    case expect_distances:
        process_distance_line( line );
        break;

    case done:
        // Shouldn' have data at this point
        m_state = data_ignored;
        break;

    case data_ignored:
        // Compiler placating
        break;
    }
}

TSDFVolume * BlockTSDFLoader::to_tsdf( TSDFVolume::volume_type type ) const {

    TSDFVolume * volume = TSDFVolume::make_volume( type, m_size_x, m_size_y, m_size_z, m_psize_x, m_psize_y, m_psize_z );
    volume->set_distance_data( m_distance_data );
    volume->set_weight_data( m_weight_data );
    return volume;
}

}

