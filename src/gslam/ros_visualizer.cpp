#include "gslam/ros_visualizer.h"

namespace gSlam
{


    RosVisualizer::RosVisualizer(bool optimise): optimisation_flag_(optimise)
    {

        marker_pub_ = rosNodeHandle.advertise<visualization_msgs::Marker>("markers", 10);
        // ----- Pulblisher for publishing trajectory as Path message. If using Marker message for publishing trajectory, this is not required.
        trajectory_publisher_ = rosNodeHandle.advertise<nav_msgs::Path>("trajectory",1000);
        // ros::Rate rate_(1000);

        // ----- stam_world_points_msg_: for visualizing the world points as obtained from STAM 
        stam_world_points_msg_.id = 0;
        stam_world_points_msg_.header.frame_id = "world_frame";
        stam_world_points_msg_.ns = "3D Keypoints";
        stam_world_points_msg_.type = visualization_msgs::Marker::POINTS;
        stam_world_points_msg_.action = visualization_msgs::Marker::ADD;
        stam_world_points_msg_.color.g = 1.0f;
        stam_world_points_msg_.scale.x = 0.005;
        stam_world_points_msg_.scale.y = 0.005;
        stam_world_points_msg_.color.a = 1.0; // always keep 1.0

        // ----- updated_worldpts_msg_: visualizes the updated world points after graph optimisation
        updated_worldpts_msg_.id = 1;
        updated_worldpts_msg_.header.frame_id = "world_frame";
        updated_worldpts_msg_.ns = "Updated 3D Keypoints";
        updated_worldpts_msg_.type = visualization_msgs::Marker::POINTS;
        updated_worldpts_msg_.action = visualization_msgs::Marker::ADD;
        updated_worldpts_msg_.scale.x = 0.005;
        updated_worldpts_msg_.scale.y = 0.005;
        updated_worldpts_msg_.color.a = 1.0; // always keep 1.0
        updated_worldpts_msg_.color.r = 1.0f;
        // ------------

        // ----- trajectory msg parameters
        optimised_trajectory_msg_.id = 2;
        optimised_trajectory_msg_.header.frame_id = "world_frame";
        optimised_trajectory_msg_.ns = "Trajectory";
        optimised_trajectory_msg_.type = visualization_msgs::Marker::LINE_STRIP;
        optimised_trajectory_msg_.action = visualization_msgs::Marker::ADD;
        optimised_trajectory_msg_.scale.x = 0.005;
        optimised_trajectory_msg_.color.a = 1.0; // always keep 1.0
        optimised_trajectory_msg_.color.r = 0.0f;
        optimised_trajectory_msg_.color.g = 0.0;
        optimised_trajectory_msg_.color.b = 0.0f;
    }    

    // ----- Creates a transformstamped message to visualize the current STAM pose in rviz
    geometry_msgs::TransformStamped RosVisualizer::createOdomMsg(customtype::TransformSE3 posemat)
    {
        // get quaternions from the pose matrix

        double q1,q2,q3,q4;
        Eigen::Matrix3d rotmat = posemat.matrix().topLeftCorner(3,3);
        Eigen::Quaterniond Q(rotmat);
        Q.normalize();
        q1 = Q.coeffs()[0]; q2 = Q.coeffs()[1]; q3 = Q.coeffs()[2]; q4 = Q.coeffs()[3];

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
    geometry_msgs::TransformStamped RosVisualizer::setFrameCorrection()
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
    void RosVisualizer::createPointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& points_msg)
    {
        points_msg.header.stamp = ros::Time::now();
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
            points_msg.points.push_back (gm_p);
        }
    }


    // ---------- Create path using marker message
    void RosVisualizer::createOptimisedTrajectoryMsg(DataSpot3D::DataSpotMap posemap, visualization_msgs::Marker& optimised_trajectory_msg)
    {

        optimised_trajectory_msg.header.stamp = ros::Time::now();
        optimised_trajectory_msg.points.clear();

        // creating path from all the dataspots in the datapool
        for (auto it = posemap.begin(); it != posemap.end(); it++)
        {
            Eigen::Vector3d t = it->second->getPose().translation();
            geometry_msgs::Point pose_pt;
            pose_pt.x = t.x()/visualization_scale_;
            pose_pt.y = t.y()/visualization_scale_;
            pose_pt.z = t.z()/visualization_scale_;
            optimised_trajectory_msg.points.push_back(pose_pt);

        }
    }

    // --------- Create trajectory using path message
    nav_msgs::Path RosVisualizer::createPathMsg(DataSpot3D::DataSpotMap posemap)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world_frame";
        path_msg.header.stamp = ros::Time::now();

        std::vector<geometry_msgs::PoseStamped> poses(posemap.size());

        // creating path from all the dataspots in the datapool
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

    // ----- checks if the poses have changed from the original STAM poses. If yes, new world points are obtained and they are used to create marker message. (Not a good method if there is large rotational correction in map)
    void RosVisualizer::checkMapUpdateAndCreateNewPointMsg(DataSpot3D::DataSpotMap pool, visualization_msgs::Marker& points_msg)
    {

        bool changed = false;

        // ----- If the poses have changed, the transformed points will be stored in this object
        customtype::WorldPtsType transformed_points;
        for (auto it = original_poses_.begin(); it != original_poses_.end(); it++)
        {
            // ----- getting the original pose from original_poses_ and the corresponding point from the datapool
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

                // customtype::TransformSE3 pose_change = original_pose.inverse()*new_pose;

                customtype::WorldPtsType world_points = spot->getWorldPoints();

                // ----- transforming points according to change in pose
                for (int i = 0; i < world_points.size(); i++) 
                {
                    cv::Point3f pt = world_points[i];

                    // // ----- finding the distance of the points from the original pose along the x, y and z directions
                    float dx = pt.x - original_position.x();
                    float dy = pt.y - original_position.y();
                    float dz = pt.z - original_position.z();

                    // ----- creating new points that are the same distance from the new pose
                    cv::Point3f new_pt(new_position.x() + dx, new_position.y() + dy, new_position.z() + dz);
                    transformed_points.push_back(new_pt);

                    // Eigen::Vector4d pt(world_points[i].x, world_points[i].y, world_points[i].z, 1.0);
                    // Eigen::Vector4d  new_pt_pose = pose_change*pt;

                    // std::cout << pt << std::endl << std::endl << new_pt_pose << std::endl;

                    // float scale = new_pt_pose[3];
                    // cv::Point3f new_pt(new_pt_pose[0]/scale, new_pt_pose[1]/scale, new_pt_pose[2]/scale);
                    // transformed_points.push_back(new_pt);
                }
            }
        }

        if (changed)
        {
            createPointMsg(transformed_points, points_msg);  
        }
    }


    void RosVisualizer::updateRosMessagesAndPublish(customtype::WorldPtsType world_points, DataSpot3D::DataSpotMap pool, int frame_no, customtype::TransformSE3 posemat)
    {
        // ros::Rate rate_(1000);
        ros::spinOnce();
        // -------- update stam_world_points_msg_ only when new world points are observed by STAM
        if (world_points.size()>0)
        {
            createPointMsg(world_points, stam_world_points_msg_);
            if (optimisation_flag_)
            {
                storeTruePose(frame_no, posemat);
                checkMapUpdateAndCreateNewPointMsg(pool, updated_worldpts_msg_);
            }
        }

        geometry_msgs::TransformStamped odom_trans = createOdomMsg(posemat);

        //// ------ Use 1 of the following trajectory message types
        // createOptimisedTrajectoryMsg(pool, optimised_trajectory_msg_);
        path_msg = createPathMsg(pool);
        //// --------------------------------------------

        // -------- publish the transform and world points
        odom_broadcaster_.sendTransform(odom_trans);
        marker_pub_.publish(stam_world_points_msg_);
        if (optimisation_flag_)
        {
            marker_pub_.publish(updated_worldpts_msg_);
        }
        // marker_pub_.publish(optimised_trajectory_msg_); // for publishing trajectory using markers

        //// ------ Use only if publishing path message and not marker message for trajectory
        trajectory_publisher_.publish(path_msg);
        //// ----------------------------
        // rate_.sleep();
    }

}// gSlam