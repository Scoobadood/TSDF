#include "../include/DepthImage.hpp"
#include "../include/PngUtilities.hpp"
#include "../include/FileUtilities.hpp"

#include <stdexcept>

DepthImage::DepthImage( std::string file_name ) {
	m_data = nullptr;

	// Check file exists
	bool is_directory;
	if ( file_exists( file_name, is_directory ) && !is_directory ) {
		uint32_t w, h;
		m_data = load_png_from_file(file_name, w, h);


		if ( m_data != nullptr) {
			m_width = w;
			m_height = h;
		} else {
			throw std::invalid_argument( "Problem reading depth image " + file_name);
		}
	} else {
		throw std::invalid_argument( "File not found or is directory " + file_name);
	}
}

/**
 * Construct a Depthimage from an array of uint16 data
 */
DepthImage::DepthImage( const uint16_t width, const uint16_t height, const uint16_t * const data ) {
	if( width > 0 && height > 0 && data != nullptr ) {
		m_data = new uint16_t[width * height];
		if( m_data ) {
			m_width = width;
			m_height = height;
			memcpy( m_data, data, width*height*sizeof( uint16_t));
		} else {
			throw std::bad_alloc( );
		}
	} else {
		throw std::invalid_argument( "width and height must be non-zero and data must not be null");
	}
}


DepthImage::~DepthImage( ) {
	std::cout<< "Destroyed depth image" << std::endl;
	if( m_data ) {
		delete [] m_data;
		m_data = nullptr;
	} else {
		std::cout << "m_data is null" << std::endl;
	}
}

/**
 * Scale the depth values by some factor
 * @param factor The factor
 */
void DepthImage::scale_depth( const float factor ) {
	size_t num_entries = m_width * m_height;

	if( m_data ) {
		for( int i=0; i<num_entries; i++ ) {
			m_data[i] = (uint16_t)( (float) m_data[i] * factor);
		}
	}
}

/**
 * Truncate depths to some particular value in mm
 * @param max_depth
 */
void DepthImage::truncate_depth_to( const int mm ) {
	size_t num_entries = m_width * m_height;
	if( m_data ) {
		for( int i=0; i<num_entries; i++ ) {
			if( m_data[i] > mm ) {
				m_data[i] = 0;
			}
		}
	}
}

/**
 * Extract the min and max values of the depth data in mm
 */
void DepthImage::min_max( uint16_t& min, uint16_t& max ) {
	min = 0xFFFF;
	max = 0;	
	size_t num_entries = m_width * m_height;
	if( m_data ) {
		for( int i=0; i<num_entries; i++ ) {
			uint16_t v = m_data[i];
			if( v > max ) max = v;
			if( v < min ) min = v;
		}
	}
}

/**
 * @return the width of the depth image
 */
uint16_t DepthImage::width( ) const {
	return m_width;
}

/**
 * @return the height of the depth image
 */
uint16_t DepthImage::height( ) const {
	return m_height;
}

/**
 * @return a pointer to the data of the image
 */
const uint16_t * DepthImage::data( ) const {
	return m_data;
}
