#ifndef SRSF_MOCK_SCENE_FLOW_ALGORITHM_HPP
#define SRSF_MOCK_SCENE_FLOW_ALGORITHM_HPP

#include "MockSceneFlowAlgorithm.hpp"

#include "tinyxml.h"

#include <vector>


class SRSFMockSceneFlowAlgorithm : public MockSceneFlowAlgorithm {
public:
	SRSFMockSceneFlowAlgorithm( const std::string & sceneFlowDirectoryName ) : MockSceneFlowAlgorithm{ sceneFlowDirectoryName } {}

private:
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
	virtual bool read_scene_flow( const std::string & file_name, 
							Eigen::Vector3f& translation, 
							Eigen::Vector3f& rotation, 
							Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals);

	virtual bool is_matched( const std::string& name );
};

#endif