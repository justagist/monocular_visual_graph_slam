#ifndef __ROS_UTILS__
#define __ROS_UTILS__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include "gslam/typedefs.h"
#include <visualization_msgs/Marker.h>
#include "gslam/data_pool.h"
#include <nav_msgs/Path.h>
#include <algorithm>

namespace gSlam
{
    class worldpt_struct
    {   
    public:
        int x, y, z;
        
        friend bool operator== ( const worldpt_struct &n1, const worldpt_struct &n2);
        
    };

    bool operator== ( const worldpt_struct &n1, const worldpt_struct &n2)
    {
        return (n1.x == n2.x && n1.y == n2.y && n1.z == n2.z);
    }

    class RosVisualizer
    {

    public:

        RosVisualizer(bool optimise = false, bool ismar_coordinates = false);
        ~RosVisualizer(){ros::shutdown();}

        void updateRosMessagesAndPublish(customtype::WorldPtsType world_points, DataSpot3D::DataSpotMap pool, int frame_no, customtype::TransformSE3 posemat, customtype::KeyPoints kpts, cv::Mat src_frame, customtype::WorldPtsType current_world_points);

    private:

        // ==========================================

        const int visualization_scale_ = 1000; // for better visualization in rviz
        const float virtual_map_scale_ = 0.003; // for scaling point-to-point when creating point cloud for pixels near a keypoints (in createVirtualMap)

        std::map<customtype::Identifier, customtype::TransformSE3> original_poses_; // stores original poses of the frames which bring new world points. These frames bring the points that create the 3D map. Updating these points according to the change in the poses after graph optimization should give the updated map.

        bool optimisation_flag_;
        bool use_ismar_coordinates_;
        std::vector<worldpt_struct> all_world_pts_;

        std::vector<worldpt_struct> point_map_structs_;

        struct PointMsgBlock
        {
            unsigned int from_, to_;            
        };

        std::map < unsigned int, PointMsgBlock > point_map_blocks_;
        std::map < unsigned int, unsigned int > frame_block_pair_;
        unsigned int point_block_count_;

        customtype::Mutex mutex_viz_;
                
        // ==========================================

        ros::NodeHandle rosNodeHandle;

        tf::TransformBroadcaster odom_broadcaster_;
        // tf::TransformBroadcaster frame_corrector; // coordinate frame orientation correction for ISMAR dataset -- NOT DONE CORRECTLY YET.
        ros::Publisher marker_pub_, trajectory_publisher_; // visualizing 3d worldpoints detected by STAM (can also be used for publishing (optimised) trajectory using markers). Trajectory publisher using path msg.
        visualization_msgs::Marker correct_map_points_msg_, optimised_trajectory_msg_, point_map_error_msgs_, virtual_map_msg_; // 'optimised_trajectory_msg' is used only if marker message is used for publishing trajectory.
        nav_msgs::Path path_msg;

        // =========================================

        geometry_msgs::TransformStamped createOdomMsg(customtype::TransformSE3 posemat);

        geometry_msgs::TransformStamped setFrameCorrection();

        void createVirtualMap(customtype::WorldPtsType world_points, customtype::KeyPoints kps, visualization_msgs::Marker& points, cv::Mat src, customtype::TransformSE3 cam_pose);
        void createVirtualMap2(customtype::WorldPtsType world_points, customtype::KeyPoints kps, visualization_msgs::Marker& points, cv::Mat src);

        void addNewPointsToMap(customtype::WorldPtsType current_world_pts);

        void createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer);
        void verifyAndCreatePointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer);

        // Either of the 2 following methods can be used for visualising trajectory
        void createOptimisedTrajectoryMsg(DataSpot3D::DataSpotMap posemap, visualization_msgs::Marker& optimised_trajectory_msg);
        nav_msgs::Path createPathMsg(DataSpot3D::DataSpotMap posemap);

        bool checkMapForUpdate(DataSpot3D::DataSpotMap pool, std::vector<int>& original_pose_ids, std::vector<int>& block_ids, std::vector<customtype::TransformSE3>& new_poses);

        void updatePointMap(std::vector<int> original_pose_ids, std::vector<int> block_ids, std::vector<customtype::TransformSE3> new_poses);

        void checkMapUpdateAndCreateNewPointMsg(DataSpot3D::DataSpotMap dataspots, visualization_msgs::Marker& points_msg);

        void storeTruePose(customtype::Identifier i, customtype::TransformSE3 pose) // stores true poses for the frames that bring new STAM keypoints
        {
            original_poses_.insert( std::make_pair(i, pose));
            frame_block_pair_.insert(std::make_pair(i, point_block_count_ - 1 ));
        }



    }; // RosVisualizer

    
}// gSlam


#endif // __ROS_UTILS__