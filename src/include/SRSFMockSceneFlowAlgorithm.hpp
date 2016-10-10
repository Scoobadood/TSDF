#ifndef SRSF_MOCK_SCENE_FLOW_ALGORITHM_HPP
#define SRSF_MOCK_SCENE_FLOW_ALGORITHM_HPP

#include "SceneFlowAlgorithm.hpp"

#include "tinyxml.h"

#include <vector>


class SRSFMockSceneFlowAlgorithm : public SceneFlowAlgorithm {
public:
	SRSFMockSceneFlowAlgorithm( const std::string & sceneFlowDirectoryName );


	/**
	 * Here to satisfy the interface but the deptha dn colour images are not used.
	 */
	virtual void compute_scene_flow(const DepthImage * 						 p_depth_image, 
									const PngWrapper * 						 p_colour_image,
						   			Eigen::Vector3f&   						 translation, 
						   			Eigen::Vector3f&   						 rotation,
						   			Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals );

private:
	/**
	 * Given a string, parse it into the specified number of floats
	 * @param string The source string
	 * @param numFloats The number of floats to parse
	 * @param readValues A pointer into which to store the values of floats read
	 * @return true if the number of floats werer read successfully, otherwise false
	 */
	bool read_floats_from_string( const char * string, uint num_floats, float * read_values);

	/**
	 * Read the residuals node from an SRSF xml document
	 * @param doc Teh document
	 * @param nodeName The name of the node to read (SFx, SFy or SFz usually)
	 * @param width The number of columns in the data
	 * @param height The number of rows in the data
	 * @return true if the number of floats werer read successfully, otherwise false
	 */
	float * read_residuals_node( const TiXmlDocument & doc, const char * node_name, uint32_t & width, uint32_t & height );

	/**
	 * Read the scene flow data from the given file
	 * @param fileName The name of XML file
	 * @param translation The global translation. Set by this method
	 * @param rotation The global translation. Set by this method
	 * @param residuals The residual translation per pixel
	 * @return true if the data were read correctly
	 */
	bool read_scene_flow( const std::string & file_name, Eigen::Vector3f& translation, Eigen::Vector3f& rotation, Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals);

	/** Directory containing the scene flow output XML files */
	std::string					m_directory;

	/** Vector of matched file names, sorted */
	std::vector<std::string>	m_scene_flow_file_names;

	// Index of the current file to read */
	uint						m_current_file_index;
};

#endif