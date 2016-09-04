#ifndef MOCK_KINECT_HPP
#define MOCK_KINECT_HPP

#include <string>
#include <vector>
#include "DataLoader/DepthImage.hpp"
#include "Utilities/PngWrapper.hpp"

#include "RGBDDevice.hpp"

class MockKinect : public RGBDDevice {
	public:
		/**
		 * @param directory Place from which to load colour and depth images
		 */
		MockKinect( const std::string directory );

	/**
	 * Initialise the device
	 */
	virtual void initialise( );

	/**
	 * Start to produce image pairs to call back
	 */
	virtual void start( );

	/**
	 * Stop producing image pairs
	 */
	virtual void stop( );

private:
	/** */
	std::string 		m_directory;

	/** Colour Image Files */
	std::vector<std::string>		m_colour_file_names{};

	/** Depth Image Files */
	std::vector<std::string>		m_depth_file_names{};
};

#endif
