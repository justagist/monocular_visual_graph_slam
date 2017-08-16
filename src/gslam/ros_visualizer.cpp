#include "gslam/ros_visualizer.h"

namespace gSlam
{


    RosVisualizer::RosVisualizer(bool optimise, bool ismar_coordinates): optimisation_flag_(optimise), use_ismar_coordinates_(ismar_coordinates), point_block_count_(0)
    {

        marker_pub_ = rosNodeHandle.advertise<visualization_msgs::Marker>("markers", 10);
        // ----- Pulblisher for publishing trajectory as Path message. If using Marker message for publishing trajectory, this is not required.
        trajectory_publisher_ = rosNodeHandle.advertise<nav_msgs::Path>("trajectory",1000);
        // ros::Rate rate_(1000);

        // ----- correct_map_points_msg_: for visualizing the world points as obtained from STAM 
        correct_map_points_msg_.id = 0;
        correct_map_points_msg_.header.frame_id = "world_frame";
        correct_map_points_msg_.ns = "3D Keypoints";
        correct_map_points_msg_.type = visualization_msgs::Marker::POINTS;
        correct_map_points_msg_.action = visualization_msgs::Marker::ADD;
        correct_map_points_msg_.color.g = 1.0f;
        correct_map_points_msg_.scale.x = 0.0075;
        correct_map_points_msg_.scale.y = 0.0075;
        correct_map_points_msg_.color.a = 1.0; // always keep 1.0
        correct_map_points_msg_.points.clear();

        // ----- point_map_error_msgs_: visualizes the updated world points after graph optimisation
        point_map_error_msgs_.id = 1;
        point_map_error_msgs_.header.frame_id = "world_frame";
        point_map_error_msgs_.ns = "Updated 3D Keypoints";
        point_map_error_msgs_.type = visualization_msgs::Marker::POINTS;
        point_map_error_msgs_.action = visualization_msgs::Marker::ADD;
        point_map_error_msgs_.scale.x = 0.005;
        point_map_error_msgs_.scale.y = 0.005;
        point_map_error_msgs_.color.a = 1.0; // always keep 1.0
        point_map_error_msgs_.color.r = 1.0f;
        point_map_error_msgs_.points.clear();
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
        optimised_trajectory_msg_.points.clear();

        // ----- virtual map parameters
        virtual_map_msg_.id = 2;
        virtual_map_msg_.header.frame_id = "world_frame";
        virtual_map_msg_.ns = "Virtual Map";
        virtual_map_msg_.type = visualization_msgs::Marker::POINTS;
        virtual_map_msg_.action = visualization_msgs::Marker::ADD;
        virtual_map_msg_.scale.x = 0.005;
        virtual_map_msg_.scale.y = 0.005;
        // virtual_map_msg_.color.a = 1.0; // always keep 1.0
        virtual_map_msg_.points.clear();

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

        odom_trans.transform.translation.x = posemat.translation()[0]/visualization_scale_;
        odom_trans.transform.translation.y = posemat.translation()[1]/visualization_scale_;
        odom_trans.transform.translation.z = posemat.translation()[2]/visualization_scale_;

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

    // ----- creates a map using the pixels around the image keypoints whose 3D position in the world have been triangulated and obtained. Memory-expensive.
    void RosVisualizer::createVirtualMap(customtype::WorldPtsType current_world_pts, customtype::KeyPoints kpts, visualization_msgs::Marker& points_msg, cv::Mat src, customtype::TransformSE3 cam_pose)
    {
        points_msg.header.stamp = ros::Time::now();
        std_msgs::ColorRGBA crgb; // for defining the color of each point

        std::cout << "size of new VirtualMap points: " << current_world_pts.size() << " " << kpts.size() << std::endl;
        assert(kpts.size() == current_world_pts.size());

        float scale; // for adjusting the 3D point distance between the nearby pixels for each keypoint. This should increase with increase in distance from the camera

        auto it_img = kpts.begin();
        for(auto it = current_world_pts.begin(); it != current_world_pts.end(); it++)
        {
            cv::Point3d point = *it;
            worldpt_struct pt_struct;
            pt_struct.x = point.x;
            pt_struct.y = point.y;
            pt_struct.z = point.z;
            if (std::find(all_world_pts_.begin(), all_world_pts_.end(), pt_struct) == all_world_pts_.end())
            {

                all_world_pts_.push_back(pt_struct);

                cv::KeyPoint kpt = *it_img;

                Eigen::Vector4d e_pt(point.x, point.y, point.z, 1);

                // ----- transforming points from ismar frame to the GSlam world frame
                if (use_ismar_coordinates_)
                    e_pt = SlamParameters::ismar_frame_aligner_*e_pt;

                // ----- transforming points to the camera coordinate frame so that new points can be created in the nearby region of each keypoint in the direction of the camera view
                Eigen::Vector4d tf_pt =  cam_pose.inverse()*e_pt;
                
                // ----- adjusting scale so that world points that are farther from the camera have more inter-marker spacing while visualizing (If the point is far, pixel distance is more significant in world coordinates) 
                if (use_ismar_coordinates_)
                    scale = (tf_pt(2)/tf_pt(3))*virtual_map_scale_*0.5; 
                else scale = (tf_pt(0)/tf_pt(3))*virtual_map_scale_;

                // ----- Window around the keypoint is selected
                for (int px = -5; px < 6; px++)
                {
                    for (int py = -5; py < 6; py++)
                    {
                        geometry_msgs::Point gm_p;

                        // ----- assiging the point the same color as the corresponding image pixel 
                        cv::Vec3b intensity = src.at<cv::Vec3b>(kpt.pt.y + py, kpt.pt.x + px);
                        crgb.r = intensity.val[2] / 255.0;
                        crgb.g = intensity.val[1] / 255.0;
                        crgb.b = intensity.val[0] / 255.0;
                        crgb.a = 1.0;

                        // ----- The region will be placed around the known keypoint, such that it is perpendicular to the camera axis. The ISMAR camera has its z axis the camera axis and the drone has its x-axis
                        Eigen::Vector4d pt_in_kpt_cloud;
                        if (use_ismar_coordinates_)
                            pt_in_kpt_cloud << (tf_pt(0)/tf_pt(3))+(px*scale), (tf_pt(1)/tf_pt(3))+(py*scale), tf_pt(2)/tf_pt(3), 1;
                        else pt_in_kpt_cloud << (tf_pt(0)/tf_pt(3)), (tf_pt(1)/tf_pt(3))-(px*scale), tf_pt(2)/tf_pt(3)-(py*scale), 1;


                        Eigen::Vector4d pt_in_original_frame = cam_pose*pt_in_kpt_cloud;

                        // ----- Converting the point back to the world frame 
                        gm_p.x = (pt_in_original_frame(0)/pt_in_original_frame(3))/visualization_scale_; gm_p.y = (pt_in_original_frame(1)/pt_in_original_frame(3))/visualization_scale_; gm_p.z = (pt_in_original_frame(2)/pt_in_original_frame(3))/visualization_scale_;

                        points_msg.points.push_back (gm_p);
                        points_msg.colors.push_back(crgb);
                    }
                }
            }

            ++it_img;
        }
        if (all_world_pts_.size() > 2000)
        {
            std::vector<worldpt_struct>::const_iterator first = all_world_pts_.end() - 3000;
            std::vector<worldpt_struct>::const_iterator last = all_world_pts_.end();
            std::vector<worldpt_struct> new_vec(first,last);
            all_world_pts_.clear();
            all_world_pts_ = new_vec;
        }

    }

    // ----- Creates marker points which mimic the color of the corresponding keypoint pixel in the image. Requires less memory.
    void RosVisualizer::createVirtualMap2(customtype::WorldPtsType current_world_pts, customtype::KeyPoints kpts, visualization_msgs::Marker& points_msg, cv::Mat src)
    {
        points_msg.header.stamp = ros::Time::now();
        auto it_img = kpts.begin();
        std_msgs::ColorRGBA crgb;
        // std::cout << "img size \n " << src.size() << std::endl;
        std::cout << "size of vectors: " << current_world_pts.size() << " " << kpts.size() << std::endl;
        assert(kpts.size() == current_world_pts.size());
        for(auto it = current_world_pts.begin(); it != current_world_pts.end(); it++)
        {
            cv::Point3d point = *it;
            geometry_msgs::Point gm_p;

            cv::KeyPoint kpt = *it_img;
            // std::cout << kpt.pt << std::endl;
            cv::Vec3b intensity = src.at<cv::Vec3b>(kpt.pt.y, kpt.pt.x);
            crgb.r = intensity.val[2] / 255.0;
            crgb.g = intensity.val[1] / 255.0;
            crgb.b = intensity.val[0] / 255.0;
            crgb.a = 1.0;

            //// ismar --------------
            if (use_ismar_coordinates_)
            {
                gm_p.x = -point.z/visualization_scale_; gm_p.y = point.x/visualization_scale_; gm_p.z = -point.y/visualization_scale_;
            }
            //// --------------------

            //// actual -------------
            else
            {
                gm_p.x = point.x/visualization_scale_; gm_p.y = point.y/visualization_scale_; gm_p.z = point.z/visualization_scale_;
            }
            //// --------------------
            points_msg.points.push_back (gm_p);
            points_msg.colors.push_back(crgb);
            ++it_img;

        }

    }

    void RosVisualizer::addNewPointsToMap(customtype::WorldPtsType worldpts)
    {
        unsigned int from, to;

        from = correct_map_points_msg_.points.size();

        verifyAndCreatePointMsg(worldpts, correct_map_points_msg_);

        to = correct_map_points_msg_.points.size();

        std::cout << from << " " << to << std::endl;

        if (optimisation_flag_)
        {
            PointMsgBlock point_block;
            point_block.from_ = from;
            point_block.to_ = to;

            point_map_blocks_.insert(std::make_pair(point_block_count_++, point_block));
        }
    }

    // ----- verifies that the points have not been added previously to the message
    void RosVisualizer::verifyAndCreatePointMsg(customtype::WorldPtsType world_points, visualization_msgs::Marker& points_msg)
    {
        points_msg.header.stamp = ros::Time::now();
        for(auto it = world_points.begin(); it != world_points.end(); it++)
        {
            cv::Point3d point = *it;
            worldpt_struct pt_struct;
            pt_struct.x = point.x;
            pt_struct.y = point.y;
            pt_struct.z = point.z;

            if (std::find(point_map_structs_.begin(), point_map_structs_.end(), pt_struct) == point_map_structs_.end())
            {

                point_map_structs_.push_back(pt_struct);
                geometry_msgs::Point gm_p;


                //// ismar --------------
                if (use_ismar_coordinates_)
                {
                    Eigen::Vector4d pt(point.x, point.y, point.z,1);
                    Eigen::Vector4d new_pt = SlamParameters::ismar_frame_aligner_*pt;
                    gm_p.x = new_pt(0)/visualization_scale_; gm_p.y = new_pt(1)/visualization_scale_; gm_p.z = new_pt(2)/visualization_scale_;
                }
                //// --------------------

                //// actual -------------
                else
                {
                    gm_p.x = point.x/visualization_scale_; gm_p.y = point.y/visualization_scale_; gm_p.z = point.z/visualization_scale_;
                }
                //// --------------------
                points_msg.points.push_back (gm_p);
            }
        }

        // ----- clearing long-term memory
        if (point_map_structs_.size() > 2000)
        {
            std::vector<worldpt_struct>::const_iterator first = point_map_structs_.end() - 3000;
            std::vector<worldpt_struct>::const_iterator last = point_map_structs_.end();
            std::vector<worldpt_struct> new_vec(first,last);
            point_map_structs_.clear();
            point_map_structs_ = new_vec;
        }
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
            if (use_ismar_coordinates_)
            {
                Eigen::Vector4d pt(point.x, point.y, point.z,1);
                Eigen::Vector4d new_pt = SlamParameters::ismar_frame_aligner_*pt;
                gm_p.x = new_pt(0)/visualization_scale_; gm_p.y = new_pt(1)/visualization_scale_; gm_p.z = new_pt(2)/visualization_scale_;
            }
            //// --------------------

            //// actual -------------
            else
            {
                gm_p.x = point.x/visualization_scale_; gm_p.y = point.y/visualization_scale_; gm_p.z = point.z/visualization_scale_;
            }
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

                customtype::TransformSE3 pose_change = original_pose.inverse()*new_pose;

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

                    // Eigen::Vector4d pt(world_points[i].x, world_points[i].y, world_points[i].z, 1.0);
                    // Eigen::Vector4d new_pt_pose = original_pose.inverse()*pt;
                    // Eigen::Vector4d new_pt_pose1 = pose_change*new_pt_pose;
                    // new_pt_pose = new_pose*new_pt_pose1;

                    // // std::cout << pt << std::endl << std::endl << new_pt_pose << std::endl;

                    // float scale = new_pt_pose[3];
                    // cv::Point3f new_pt(new_pt_pose[0]/scale, new_pt_pose[1]/scale, new_pt_pose[2]/scale);
                    // transformed_points.push_back(new_pt);
                }
            }
        }

        if (changed)
        {
            points_msg.points.clear();
            createPointMsg(transformed_points, points_msg);  
        }
    }

    bool RosVisualizer::checkMapForUpdate(DataSpot3D::DataSpotMap pool, std::vector<int>& original_pose_ids, std::vector<int>& block_ids, std::vector<customtype::TransformSE3>& new_poses)
    {
        bool changed = false;
        for (auto it = original_poses_.begin(); it != original_poses_.end(); it++)
        {
            // ----- getting the original pose from original_poses_ and the corresponding point from the datapool
            customtype::TransformSE3 original_pose = it->second;
            DataSpot3D::DataSpot3DPtr spot = pool.find(it->first)->second;
            customtype::TransformSE3 new_pose = spot->getPose();

            Eigen::Vector3d original_position = original_pose.translation();
            Eigen::Vector3d new_position = new_pose.translation();

            float dist = (original_position-new_position).norm();

            // ----- checking if the poses have changed enough to trigger new worldpoints computation
            if (dist >= 10.0)
            {
                changed = true;
                original_pose_ids.push_back(it->first);
                new_poses.push_back(new_pose);
                block_ids.push_back(frame_block_pair_.find(it->first)->second);
            }
        }

        return changed;
    }

    void RosVisualizer::updatePointMap(std::vector<int> original_pose_ids, std::vector<int> block_ids, std::vector<customtype::TransformSE3> new_poses)
    {

        int block_count = 0;
        for (std::vector<int>::iterator it = block_ids.begin(); it != block_ids.end(); ++it)
        {

            Eigen::Vector3d original_position = original_poses_.find(original_pose_ids[block_count])->second.translation();
            customtype::TransformSE3 new_pose = new_poses[block_count];
            Eigen::Vector3d new_position = new_pose.translation();

            // std::cout << "actual pose: " << original_position <<" new_pose: " << new_position << std::endl;

            // ----- selecting the block of world points associated with the original pose
            PointMsgBlock block = point_map_blocks_.find(*it)->second;

            // std::cout << block.from_ << " " << block.to_ << std::endl;

            // std::cout << correct_map_points_msg_.points.size() << std::endl;

            for (int i = block.from_; i < block.to_; ++i)
            {
                geometry_msgs::Point original_point = correct_map_points_msg_.points[i];
                geometry_msgs::Point new_pt;
                float dx = original_point.x - original_position.x();
                float dy = original_point.y - original_position.y();
                float dz = original_point.z - original_position.z();

                // ----- adding the original point to the error marker msg
                point_map_error_msgs_.points.push_back(original_point);

                // ----- defining the new position of the points according to the translation in the camera poses
                new_pt.x = new_position.x() + dx; new_pt.y = new_position.y() + dy; new_pt.z = new_position.z() + dz;

                // ----- replace the original point with the corrected point in the correct map point msg
                correct_map_points_msg_.points.at(i) = new_pt;

            }

            original_poses_[original_pose_ids[block_count]] = new_pose;


            ++block_count;
        }

    }


    void RosVisualizer::updateRosMessagesAndPublish(customtype::WorldPtsType world_points, DataSpot3D::DataSpotMap pool, int frame_no, customtype::TransformSE3 posemat, customtype::KeyPoints kpts, cv::Mat src_frame, customtype::WorldPtsType current_world_pts)
    {
        ros::spinOnce();
        // -------- update correct_map_points_msg_ only when new world points are observed by STAM.
        if (world_points.size()>0)
        {
            // ----- create a virtual map using the world points and the color of the corresponding image points
            // createVirtualMap(current_world_pts, kpts, virtual_map_msg_, src_frame, posemat);

            // ----- create the point markers for the 3D points obtained from STAM
            addNewPointsToMap(world_points);
            // createPointMsg(world_points, correct_map_points_msg_);

            if (optimisation_flag_)
            {
                {
                    // mutex_viz_.lock();
                    // ----- storing true poses to later compare if the poses changed due to graph optimisation
                    storeTruePose(frame_no, posemat);
                    // mutex_viz_.unlock();

                    std::vector<int> original_pose_ids, point_block_ids;
                    std::vector<customtype::TransformSE3> new_posemats;
                    
                    // ----- checks if the poses have changed, if yes, gives the changed poses
                    // mutex_viz_.lock();
                    bool map_changed = checkMapForUpdate(pool, original_pose_ids, point_block_ids, new_posemats);
                    // mutex_viz_.unlock();

                    // ----- and corrects map if true
                    if (map_changed)
                    {
                        // {
                            // customtype::Lock lk(mutex_viz_);
                            updatePointMap(original_pose_ids, point_block_ids, new_posemats);
                        // }

                    }

                    // checkMapUpdateAndCreateNewPointMsg(pool, point_map_error_msgs_);
                }
            }

        }

        geometry_msgs::TransformStamped odom_trans = createOdomMsg(posemat);

        //// ------ Use 1 of the following trajectory message types
        // createOptimisedTrajectoryMsg(pool, optimised_trajectory_msg_);
        path_msg = createPathMsg(pool);
        //// --------------------------------------------

        // -------- publish the transform and world points
        odom_broadcaster_.sendTransform(odom_trans);
        marker_pub_.publish(correct_map_points_msg_);
        // marker_pub_.publish(virtual_map_msg_);
        if (optimisation_flag_)
        {
            marker_pub_.publish(point_map_error_msgs_);
        }
        // marker_pub_.publish(optimised_trajectory_msg_); // for publishing trajectory using markers

        //// ------ Use only if publishing path message and not marker message for trajectory
        trajectory_publisher_.publish(path_msg);
        //// ----------------------------
        // rate_.sleep();
    }

}// gSlam