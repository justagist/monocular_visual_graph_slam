
#include "gslam/data_spot.h"

namespace gSlam
{

DataSpot3D::DataSpot3D(const customtype::TransformSE3& pose, 
                       const CameraParameters& camParams, 
                       const cv::Mat& image_color) : pose_(pose), id_(next_id_s++), added_to_graph_(false), camParams_(camParams), image_color_(image_color)
{
}

customtype::Identifier DataSpot3D::next_id_s = 0;

}