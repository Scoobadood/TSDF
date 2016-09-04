#ifndef SRSF_MOCK_SCENE_FLOW_ALGORITHM_HPP
#define SRSF_MOCK_SCENE_FLOW_ALGORITHM_HPP

#include "SceneFlowAlgorithm.hpp"

#include "tinyxml.h"

class SRSFMockSceneFlowAlgorithm : public SceneFlowAlgorithm {
public:
	SRSFMockSceneFlowAlgorithm( const std::string & sceneFlowDirectoryName );


	/**
	 * Here to satisfy the interface but the deptha dn colour images are not used.
	 */
	virtual void computeSceneFlow( const DepthImage * pDepthImage, const PngWrapper * pColourImage,
	                               Eigen::Vector3f&   						translation,
	                               Eigen::Vector3f&   						rotation,
	                               Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals );

private:
	/**
	 * Given a string, parse it into the specified number of floats
	 * @param string The source string
	 * @param numFloats The number of floats to parse
	 * @param readValues A pointer into which to store the values of floats read
	 * @return true if the number of floats werer read successfully, otherwise false
	 */
	bool readFloatsFromString( const char * string, uint num_floats, float * read_values);

	/**
	 * Read the residuals node from an SRSF xml document
	 * @param doc Teh document
	 * @param nodeName The name of the node to read (SFx, SFy or SFz usually)
	 * @param width The number of columns in the data
	 * @param height The number of rows in the data
	 * @return true if the number of floats werer read successfully, otherwise false
	 */
	float * readResidualsNode( const TiXmlDocument & doc, const char * node_name, uint32_t & width, uint32_t & height );

	/**
	 * Read the scene flow data from the given file
	 * @param fileName The name of XML file
	 * @param translation The global translation. Set by this method
	 * @param rotation The global translation. Set by this method
	 * @param residuals The residual translation per pixel
	 * @return true if the data were read correctly
	 */
	bool readSceneFlow( const std::string & file_name, Eigen::Vector3f& translation, Eigen::Vector3f& rotation, Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals);

	std::string		mDirectory;
	uint			mNumFiles;
	uint			mCurrentFileIndex;
};

#endif