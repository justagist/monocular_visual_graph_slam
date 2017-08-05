#ifndef __ROS_UTILS__
#define __ROS_UTILS__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include "gslam/typedefs.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

namespace gSlam
{
    namespace ros_utils
    {

        geometry_msgs::TransformStamped createOdomMsg(customtype::TransformSE3 posemat);

        geometry_msgs::TransformStamped setFrameCorrection();

        visualization_msgs::Marker createPointMsg(std::vector<cv::Point3f> world_points);

        nav_msgs::Path createPathMsg();

    } // ros_utils
}// gSlam


#endif // __ROS_UTILS__