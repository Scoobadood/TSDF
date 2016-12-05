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
	 * @return a pointer to the data of the image
	 */
	const uint16_t * data( ) const;

private:
	uint16_t	m_width;
	uint16_t	m_height;
	uint16_t	* m_data;
};


#endif
