#ifndef MOCK_SCENE_FLOW_ALGORITHM_HPP
#define MOCK_SCENE_FLOW_ALGORITHM_HPP

#include "SceneFlowAlgorithm.hpp"
#include <vector>

class MockSceneFlowAlgorithm : public SceneFlowAlgorithm {
protected :

	/** Directory containing the scene flow output XML files */
	std::string					m_directory;

	/** Vector of matched file names, sorted */
	std::vector<std::string>	m_scene_flow_file_names;

	// Index of the current file to read */
	uint						m_current_file_index;


	MockSceneFlowAlgorithm( const std::string & sceneFlowDirectoryName );

	/**
	 * Here to satisfy the interface but the depth and colour images are not used.
	 */
	virtual void compute_scene_flow(const DepthImage * 						 p_depth_image, 
									const PngWrapper * 						 p_colour_image,
						   			Eigen::Vector3f&   						 translation, 
						   			Eigen::Vector3f&   						 rotation,
						   			Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals );
	/**
	 * Given a string, parse it into the specified number of floats
	 * @param string The source string
	 * @param numFloats The number of floats to parse
	 * @param readValues A pointer into which to store the values of floats read
	 * @return true if the number of floats werer read successfully, otherwise false
	 */
	bool read_floats_from_string( const char * string, uint num_floats, float * read_values);

	/**
	 * Call to initialise the thing
	 * Override if you like but always call super
	 */
	virtual bool init( ) ;

	/**
	 * Read the scene flow data from the given file
	 * @param fileName The name of XML file
	 * @param translation The global translation. Set by this method
	 * @param rotation The global translation. Set by this method
	 * @param residuals The residual translation per pixel
	 * @return true if the data were read correctly
	 */
	virtual bool read_scene_flow( const std::string & file_name, 
							Eigen::Vector3f& translation, 
							Eigen::Vector3f& rotation, 
							Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals)=0;

	virtual bool is_matched( const std::string& name ) =0;
};

#endif