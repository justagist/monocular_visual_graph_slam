#include "gslam/ros_utils.h"
#include <math.h>


namespace gSlam
{


    namespace ros_utils
    {

        const int visualization_scale_ = 1000; // for better visualization in rviz

        // ----- Creates a transformstamped message to visualize the current STAM pose in rviz
        geometry_msgs::TransformStamped createOdomMsg(customtype::TransformSE3 posemat)
        {
            // get quaternions from the pose matrix

            double q1,q2,q3,q4;
            Eigen::Matrix3d rotmat = posemat.matrix().topLeftCorner(3,3);
            Eigen::Quaterniond Q(rotmat);
            Q.normalize();
            q1 = Q.coeffs()[0]; q2 = Q.coeffs()[1]; q3 = Q.coeffs()[2]; q4 = Q.coeffs()[3];
            
            /*//Alternatively get quaternion from STAM (has to be used in main.cpp. Requires same amount of calculation in STAM)

            current_odom_frame->getQuaternion(q1,q5,q3,q4);

            */

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "world_frame";
            odom_trans.child_frame_id = "cam_frame";


            // ismar coordinates ---------
            // odom_trans.transform.translation.x = -posemat.translation()[2]/visualization_scale_;
            // odom_trans.transform.translation.y = posemat.translation()[0]/visualization_scale_;
            // odom_trans.transform.translation.z = -posemat.translation()[1]/visualization_scale_;
            // --------------

            // actual coordinates ----------
            odom_trans.transform.translation.x = posemat.translation()[0]/visualization_scale_;
            odom_trans.transform.translation.y = posemat.translation()[1]/visualization_scale_;
            odom_trans.transform.translation.z = posemat.translation()[2]/visualization_scale_;

            // --------------------------

            odom_trans.transform.rotation.x = q1;
            odom_trans.transform.rotation.y = q2;
            odom_trans.transform.rotation.z = q3;
            odom_trans.transform.rotation.w = q4;
            return odom_trans;
        }

        // ----- NOT FIXED. Intented to correct the coordinate frame when using the weird ISMAR coordinate frame
        geometry_msgs::TransformStamped setFrameCorrection()
        {
            geometry_msgs::TransformStamped transf;
            transf.header.stamp = ros::Time::now();
            transf.header.frame_id = "world_frame";
            transf.child_frame_id = "world_frame_corrected";

            transf.transform.translation.x = 0;
            transf.transform.translation.y = 0;
            transf.transform.translation.z = 0;
            transf.transform.rotation.x = 0;
            transf.transform.rotation.y = 1;
            transf.transform.rotation.z = 0;
            transf.transform.rotation.w = 0;
            return transf;
        }


        // ----- Creates marker message to visualize 3D worldpoints
        void createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer)
        {
            world_visualizer.header.stamp = ros::Time::now();
            world_visualizer.action = visualization_msgs::Marker::ADD;
            world_visualizer.scale.x = 0.005;
            world_visualizer.scale.y = 0.005;
            world_visualizer.color.a = 1.0;
            for(auto it = world_points.begin(); it != world_points.end(); it++)
            {
                cv::Point3d point = *it;
                geometry_msgs::Point gm_p;
                //// ismar --------------
                // gm_p.x = -point.z/visualization_scale_; gm_p.y = point.x/visualization_scale_; gm_p.z = -point.y/visualization_scale_;
                //// --------------------

                //// actual -------------
                gm_p.x = point.x/visualization_scale_; gm_p.y = point.y/visualization_scale_; gm_p.z = point.z/visualization_scale_;
                //// --------------------
                world_visualizer.points.push_back (gm_p);
            }
        }


        // ---------- Create path using marker message
        visualization_msgs::Marker createOptimisedTrajectoryMsg(DataSpot3D::DataSpotMap posemap)
        {
            visualization_msgs::Marker optimised_trajectory_msg;
            optimised_trajectory_msg.header.frame_id = "world_frame";
            optimised_trajectory_msg.type = visualization_msgs::Marker::LINE_STRIP;
            optimised_trajectory_msg.id = 1;
            optimised_trajectory_msg.header.stamp = ros::Time::now();
            optimised_trajectory_msg.ns = "trajectory";
            optimised_trajectory_msg.action = visualization_msgs::Marker::ADD;
            optimised_trajectory_msg.scale.x = 0.005;
            optimised_trajectory_msg.color.a = 1.0; 
            optimised_trajectory_msg.color.r = 0.0f;
            optimised_trajectory_msg.color.g = 0.0;
            optimised_trajectory_msg.color.b = 0.0f;

            for (auto it = posemap.begin(); it != posemap.end(); it++)
            {
                Eigen::Vector3d t = it->second->getPose().translation();
                geometry_msgs::Point pose_pt;
                pose_pt.x = t.x()/visualization_scale_;
                pose_pt.y = t.y()/visualization_scale_;
                pose_pt.z = t.z()/visualization_scale_;
                optimised_trajectory_msg.points.push_back(pose_pt);

            }

            return optimised_trajectory_msg;
        }

        // --------- Create trajectory using path message
        nav_msgs::Path createPathMsg(DataSpot3D::DataSpotMap posemap)
        {
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "world_frame";
            path_msg.header.stamp = ros::Time::now();

            std::vector<geometry_msgs::PoseStamped> poses(posemap.size());

            for (auto it = posemap.begin(); it != posemap.end(); it++)
            {
                Eigen::Vector3d t = it->second->getPose().translation();
                poses.at(it->first).pose.position.x = t.x()/visualization_scale_;
                poses.at(it->first).pose.position.y = t.y()/visualization_scale_;
                poses.at(it->first).pose.position.z = t.z()/visualization_scale_;
                poses.at(it->first).header.frame_id = "world_frame";
                poses.at(it->first).header.stamp = ros::Time::now();
            }

            path_msg.poses = poses;
            return path_msg;
        }

        // ----- checks if the poses have changed from the original STAM poses. If yes, new world points are obtained and they are used to create marker message 
        void checkMapUpdateAndCreateNewPointMsg(DataSpot3D::DataSpotMap pool, visualization_msgs::Marker& points_msg)
        {

            bool changed = false;

            // ----- If the poses have changed, the transformed points will be stored in this object
            customtype::WorldPtsType transformed_points;
            for (auto it = storage::original_poses_.begin(); it != storage::original_poses_.end(); it++)
            {
                // ----- getting the original pose from storage and the corresponding point from the datapool
                customtype::TransformSE3 original_pose = it->second;
                DataSpot3D::DataSpot3DPtr spot = pool.find(it->first)->second;
                customtype::TransformSE3 new_pose = spot->getPose();

                Eigen::Vector3d original_position = original_pose.translation();
                Eigen::Vector3d new_position = new_pose.translation();


                float dist = (original_position-new_position).norm();
                // std::cout << dist << std::endl;

                // ----- checking if the poses have changed enough to trigger new worldpoints computation
                if (dist >= 10.0)
                {
                    changed = true;
                    customtype::TransformSE3 pose_change = original_pose.inverse()*new_pose;
                    cv::Mat_<float> cv_pose_change;
                    cv::eigen2cv(pose_change.matrix(),cv_pose_change);
                    customtype::WorldPtsType world_points = spot->getWorldPoints();


                    // ----- transforming points according to change in pose
                    for (int i = 0; i < world_points.size(); i++) 
                    {
                        cv::Point3f pt = world_points[i];

                        // ----- finding the distance of the points from the original pose along the x, y and z directions
                        float dx = pt.x - original_position.x();
                        float dy = pt.y - original_position.y();
                        float dz = pt.z - original_position.z();

                        // ----- creating new points that are the same distance from the new pose
                        cv::Point3f new_pt(new_position.x() + dx, new_position.y() + dy, new_position.z() + dz);
                        transformed_points.push_back(new_pt);
                    }
                }
            }

            if (changed)
            {
                createPointMsg(transformed_points, points_msg);                
            }
        }


    } // ros_utils
}// gSlam