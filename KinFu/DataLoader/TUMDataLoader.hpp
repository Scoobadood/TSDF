#ifndef TUM_DATA_LOADER_H
#define TUM_DATA_LOADER_H

#include "DepthImage.hpp"

#include <string>
#include <vector>
#include <Eigen/Dense>

class TUMDataLoader {

public:
	/**
	 * @param directory The dircetory from which to load data
	 */
	TUMDataLoader( const std::string& directory );

	/**
	 */
	~TUMDataLoader( );

	/**
	 * @param pose The camera pose, populated by the next method
	 * @return The next DepthImage to be read or nullptr if none
	 */
	DepthImage * next( Eigen::Matrix4f& pose );

private:
	/**
	* Create a pose matrix based on
	* 7 float parameters (as provided by TUM groundtruth data)
	* @param vars 0, 1 and 2 are a translation
	* @param vars 3,4,5,6 are x,y,z, w components of a quaternion dewscribing the facing
	*/
	Eigen::Matrix4f to_pose( float vars[7] ) const;

	void process_line( const std::string& line );
	void load_data_from( const std::string& gt_file_name );

	struct DATA_RECORD {
		std:: string	file_name;
		float 			data[7];
	};

	size_t								m_current_idx;
	std::vector<struct DATA_RECORD>		m_data_records;
	std::string							m_directory_name;
};



#endif