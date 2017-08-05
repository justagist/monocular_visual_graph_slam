#include "gslam/ros_utils.h"
#include <math.h>


namespace gSlam
{


    namespace ros_utils
    {

        const int visualization_scale_ = 1000;

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
            // std::cout << q1 << " " << q5 << " " << q3<< " " << q4 << std::endl;        

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

        void createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& world_visualizer)
        {
            // visualization_msgs::Marker world_visualizer;
            // visualization_msgs::Marker world_visualizer;
            world_visualizer.header.stamp = ros::Time::now();
            world_visualizer.action = visualization_msgs::Marker::ADD;
            world_visualizer.ns = "3D Keypoints";
            world_visualizer.scale.x = 0.005;
            world_visualizer.scale.y = 0.005;
            world_visualizer.color.g = 1.0f;
            world_visualizer.color.a = 1.0;
            for(auto it = world_points.begin(); it != world_points.end(); it++)
            {
                cv::Point3d point = *it;
                geometry_msgs::Point gm_p;
                // ismar --------------
                // gm_p.x = -point.z/visualization_scale_; gm_p.y = point.x/visualization_scale_; gm_p.z = -point.y/visualization_scale_;
                // --------------------

                // actual -------------
                gm_p.x = point.x/visualization_scale_; gm_p.y = point.y/visualization_scale_; gm_p.z = point.z/visualization_scale_;
                // --------------------
                world_visualizer.points.push_back (gm_p);
            }
        }

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

    } // ros_utils
}// gSlam