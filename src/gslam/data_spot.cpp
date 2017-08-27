/** @file dataspot.cpp (Creates an object storing all data received in a particular frame/iteration (acts like a node of the graph))
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#include "gslam/data_spot.h"

namespace gSlam
{

DataSpot3D::DataSpot3D(const customtype::TransformSE3& pose, 
                                 const CameraParameters& camParams, 
                                 const cv::Mat& image_color, 
                                 const customtype::WorldPtsType& world_pts, 
                                 const customtype::KeyPoints& img_pts) : pose_(pose), id_(next_id_s++), added_to_graph_(false), camParams_(camParams), image_color_(image_color), world_pts_(world_pts), image_pts_(img_pts)
{
    // std::cout << camParams.distortion_ << std::endl;
}

customtype::Identifier DataSpot3D::next_id_s = 0;

}