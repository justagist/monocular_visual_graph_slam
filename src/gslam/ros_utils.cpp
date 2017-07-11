#include "gslam/ros_utils.h"


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
            odom_trans.header.frame_id = "ismar_frame";
            odom_trans.child_frame_id = "cam_frame";

            odom_trans.transform.translation.x = -posemat.translation()[2]/visualization_scale_;
            odom_trans.transform.translation.y = posemat.translation()[0]/visualization_scale_;
            odom_trans.transform.translation.z = -posemat.translation()[1]/visualization_scale_;
            // std::cout << "trans " << << std::endl;
            // std::cout << "trans " << posemat.translation()[1]<< std::endl;
            // std::cout << "trans " << posemat.translation()[2]<< std::endl;
            // std::cout << "trans " << posemat.translation()<< std::endl;
            // std::cout << "Mat" << posemat.matrix() << std::endl;
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
            transf.header.frame_id = "ismar_frame";
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

        void createPointMsg(visualization_msgs::Marker& world_visualizer, std::vector<cv::Point3d> world_points)
        {
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
                    gm_p.x = -point.z/visualization_scale_; gm_p.y = point.x/visualization_scale_; gm_p.z = -point.y/visualization_scale_;
                    world_visualizer.points.push_back (gm_p);
                }
        }

    } // ros_utils
}// gSlam