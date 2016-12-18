#ifndef PDSF_MOCK_SCENE_FLOW_ALGORITHM_HPP
#define PDSF_MOCK_SCENE_FLOW_ALGORITHM_HPP

#include "MockSceneFlowAlgorithm.hpp"

#include <vector>


class PDSFMockSceneFlowAlgorithm : public MockSceneFlowAlgorithm {
public:
	PDSFMockSceneFlowAlgorithm( const std::string & sceneFlowDirectoryName ) : MockSceneFlowAlgorithm{sceneFlowDirectoryName} {}

private:
	/**
	 * Read the scene flow data from the given file
	 * @param fileName The name of XML file
	 * @param translation The global translation. Set by this method
	 * @param rotation The global translation. Set by this method
	 * @param residuals The residual translation per pixel
	 * @return true if the data were read correctly
	 */
	virtual bool read_scene_flow( const std::string & file_name, Eigen::Vector3f& translation, Eigen::Vector3f& rotation, Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals);

	virtual bool is_matched( const std::string& name );

	

};

#endif