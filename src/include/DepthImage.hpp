#ifndef DEPTH_IMAGE_H
#define DEPTH_IMAGE_H

#include <string>

class DepthImage {
public:
	/**
	 * @param file_name The name of the file from which to load the 
	 * depth image
	 */
	DepthImage( std::string file_name );

	/**
	 * Construct a Depthimage from an array of uint16 data
 	 */
	DepthImage( const uint16_t width, const uint16_t height, const uint16_t * const data );

	/**
	 * Clean up depth image memory
	 */
	~DepthImage( );


	/**
	 * Scale the depth values by some factor
	 * @param factor The factor
	 */
	void scale_depth( const float factor ) ;
	
	/**
	 * @return the width of the depth image
	 */
	uint16_t width( ) const;

	/**
	 * @return the height of the depth image
	 */
	uint16_t height( ) const;

/**
 * Truncate depths to some particular value in mm
 * @param max_depth
 */
void truncate_depth_to( const int mm );

/**
 * Extract the min and max values of the depth data in mm
 */
void min_max( uint16_t& min, uint16_t& max );

/**
	 * @return a pointer to the data of the image
	 */
	const uint16_t * data( ) const;

private:
	uint16_t	m_width;
	uint16_t	m_height;
	uint16_t	* m_data;
};


#endif
