#ifndef RGBD_DEVICE_HPP
#define RGBD_DEVICE_HPP

#include "DepthImage.hpp"
#include "PngWrapper.hpp"

#include <functional>


typedef std::function< void( const DepthImage *, const PngWrapper * )> RGBDDeviceCallback ;

/**
 * A depth imaging device
 * Responsible for delivering depth an colour images to its listener
 */
class RGBDDevice {
public:
	/**
	 * Initialise the device
	 */
	virtual void initialise( ) = 0;

	/**
	 * Start to produce image pairs to call back
	 */
	virtual void start( ) = 0;

	/**
	 * Stop producing image pairs
	 */
	virtual void stop( ) = 0;

	/**
	 * Register an observer which will be notified of new images as they arrive.
	 */
	void addObserver( RGBDDeviceCallback observer) {
		m_observer = observer;
	}

protected:
	RGBDDevice( ){};

	void notify( const DepthImage * depthImage, const PngWrapper * colourImage ) {
		if( m_observer ) {
			m_observer( depthImage, colourImage );
		}
	}


private:
	// The obserer. Notified when images arrive
	RGBDDeviceCallback 		m_observer;
};

#endif