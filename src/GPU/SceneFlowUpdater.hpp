/**
 * Update the TSDF Volume using the Scene Flow data
 * @param volume The volume
 * @param width The width of the scene flow image
 * @param height The height of the scene flow image
 * @param translation The global transform
 * @param rotation The global rotation
 * @param residuals The per pixel residual translation
 */
void update( GPUTSDFVolume& volume, uint32_t width, uint32_t height, const float3 & translation, const float3 & rotation, const float3 * residuals );