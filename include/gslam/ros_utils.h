#ifndef __ROS_UTILS__
#define __ROS_UTILS__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include "gslam/typedefs.h"
#include <visualization_msgs/Marker.h>
#include "gslam/data_pool.h"

namespace gSlam
{
    namespace ros_utils
    {

        geometry_msgs::TransformStamped createOdomMsg(customtype::TransformSE3 posemat);

        geometry_msgs::TransformStamped setFrameCorrection();

        void createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer);

        visualization_msgs::Marker createOptimisedTrajectoryMsg(DataSpot3D::DataSpotMap posemap);

    } // ros_utils
}// gSlam


#endif // __ROS_UTILS__