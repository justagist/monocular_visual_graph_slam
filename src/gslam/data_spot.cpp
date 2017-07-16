
#include "gslam/data_spot.h"

namespace gSlam
{

DataSpot3D::DataSpot3D(const customtype::TransformSE3& pose, 
                                 const CameraParameters& camParams, 
                                 const cv::Mat& image_color, 
                                 const customtype::ProjMatType& projectionMatrix, 
                                 const customtype::WorldPtsType& world_pts, 
                                 const customtype::KeyPoints& img_pts) : pose_(pose), id_(next_id_s++), added_to_graph_(false), camParams_(camParams), image_color_(image_color), projectionMat_(projectionMatrix), world_pts_(world_pts), image_pts_(img_pts)
{
}

customtype::Identifier DataSpot3D::next_id_s = 0;

}