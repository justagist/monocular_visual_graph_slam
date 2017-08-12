#ifndef __ROS_UTILS__
#define __ROS_UTILS__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include "gslam/typedefs.h"
#include <visualization_msgs/Marker.h>
#include "gslam/data_pool.h"
#include <nav_msgs/Path.h>

namespace gSlam
{
    namespace ros_utils
    {

        namespace storage // Storing stuff that are required only if using ROS (visualizing in rviz)
        {
            std::map<customtype::Identifier, customtype::TransformSE3> original_poses_;
        }



        geometry_msgs::TransformStamped createOdomMsg(customtype::TransformSE3 posemat);

        geometry_msgs::TransformStamped setFrameCorrection();

        void createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer);

        // Either of the 2 following methods can be used for visualising trajectory
        visualization_msgs::Marker createOptimisedTrajectoryMsg(DataSpot3D::DataSpotMap posemap);
        nav_msgs::Path createPathMsg(DataSpot3D::DataSpotMap posemap);

        void checkMapUpdateAndCreateNewPointMsg(DataSpot3D::DataSpotMap dataspots, visualization_msgs::Marker& points_msg);

        void storeTruePose(customtype::Identifier i, customtype::TransformSE3 pose) // stores true poses for the frames that bring new STAM keypoints
        {
            storage::original_poses_.insert( std::make_pair(i, pose));
        }

    } // ros_utils
}// gSlam


#endif // __ROS_UTILS__