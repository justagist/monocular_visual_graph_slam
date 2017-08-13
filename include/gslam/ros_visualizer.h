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

    class RosVisualizer
    {

    public:

        RosVisualizer(bool optimise = false);
        ~RosVisualizer(){ros::shutdown();}

        void updateRosMessagesAndPublish(customtype::WorldPtsType world_points, DataSpot3D::DataSpotMap pool, int frame_no, customtype::TransformSE3 posemat);

    private:

        // ==========================================

        const int visualization_scale_ = 1000; // for better visualization in rviz

        std::map<customtype::Identifier, customtype::TransformSE3> original_poses_; // stores original poses of the frames which bring new world points. These frames bring the points that create the 3D map. Updating these points according to the change in the poses after graph optimization should give the updated map.

        bool optimisation_flag_;
                
        // ==========================================

        ros::NodeHandle rosNodeHandle;

        tf::TransformBroadcaster odom_broadcaster_;
        // tf::TransformBroadcaster frame_corrector; // coordinate frame orientation correction for ISMAR dataset -- NOT DONE CORRECTLY YET.
        ros::Publisher marker_pub_, trajectory_publisher_; // visualizing 3d worldpoints detected by STAM (can also be used for publishing (optimised) trajectory using markers). Trajectory publisher using path msg.
        visualization_msgs::Marker stam_world_points_msg_, optimised_trajectory_msg_, updated_worldpts_msg_; // 'optimised_trajectory_msg' is used only if marker message is used for publishing trajectory.
        nav_msgs::Path path_msg;

        // =========================================

        geometry_msgs::TransformStamped createOdomMsg(customtype::TransformSE3 posemat);

        geometry_msgs::TransformStamped setFrameCorrection();

        void createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer);

        // Either of the 2 following methods can be used for visualising trajectory
        void createOptimisedTrajectoryMsg(DataSpot3D::DataSpotMap posemap, visualization_msgs::Marker& optimised_trajectory_msg);
        nav_msgs::Path createPathMsg(DataSpot3D::DataSpotMap posemap);

        void checkMapUpdateAndCreateNewPointMsg(DataSpot3D::DataSpotMap dataspots, visualization_msgs::Marker& points_msg);

        void storeTruePose(customtype::Identifier i, customtype::TransformSE3 pose) // stores true poses for the frames that bring new STAM keypoints
        {
            original_poses_.insert( std::make_pair(i, pose));
        }


    }; // RosVisualizer
    
}// gSlam


#endif // __ROS_UTILS__