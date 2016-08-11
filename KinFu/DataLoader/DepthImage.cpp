#include "DepthImage.hpp"
#include "../Utilities/PngUtilities.hpp"
#include "../Utilities/FileUtilities.hpp"

#include <stdexcept>

DepthImage::DepthImage( std::string file_name ) {
	m_data = NULL;

	// Check file exists
	bool is_directory;
	if ( file_exists( file_name, is_directory ) && !is_directory ) {
		uint32_t w, h;
		m_data = load_png_from_file(file_name, w, h);


		if ( m_data ) {
			m_width = w;
			m_height = h;

			size_t map_size = m_width * m_height;
			for ( size_t i = 0; i < map_size; i++ ) {
				uint16_t v = m_data[i];

				// Convert to metres by dividing by 5000, then to millimetres by multiplying by 1000
				m_data[i] = v / 5;
			}
		} else {
			throw std::invalid_argument( "Problem reading depth image " + file_name);
		}
	} else {
		throw std::invalid_argument( "File not found or is directory " + file_name);
	}
}

DepthImage::~DepthImage( ) {
	std::cout<< "Destroyed depth image" << std::endl;
	if( m_data ) {
		delete [] m_data;
		m_data = 0;
	} else {
		std::cout << "m_data is null" << std::endl;
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