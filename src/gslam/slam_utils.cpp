/** @file slam_utils.cpp (various utility functions required for graph_slam)
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

// TODO: REMOVE ALL PCL FUNCTIONS (NOT USING ANY MORE)

#include "gslam/slam_utils.h"

namespace gSlam
{

namespace slam_utils
{

    customtype::TransformSE3 getTransformation (double x, double y, double z, double roll, double pitch, double yaw)
    {
        customtype::TransformSE3 t;
        getTransformation (x, y, z, roll, pitch, yaw, t);
        return (t);
    }


    customtype::TransformSE3 getTransformation (double x, double y, double z, double qx, double qy, double qz, double qw) {

        Eigen::Quaternion<double> q;
        q.x() = qx; q.y() = qy; q.z() = qz ; q.w() = qw;
        Eigen::Vector3d t(x, y, z);
        Eigen::Translation<double, 3> trans(t);
        customtype::TransformSE3 transform = trans*q;

        return transform;

    }

    // ----- Aligns the camera frame of the ArDrone to the body frame as tracked by the mocap system (data from dataset 10)
    customtype::TransformSE3 getFrameAligner()
    {
        customtype::TransformSE3 ground_truth = getTransformation(395.2285, 598.4764, 960.3633, -0.020576022865, 0.662864768588, -0.0584081485361, 0.746173840335);
        customtype::TransformSE3 cam_frame = getTransformation(335.484, 633.562, 725.732, -0.686232, 0.716337, -0.115044, 0.0520781);
        return cam_frame.inverse()*ground_truth;

    }

    customtype::TransformSE3 getIsmarFrameAligner()
    {
        customtype::TransformSE3 alignment = customtype::TransformSE3::Identity();
        alignment(0,0) = 1; alignment(0,1) = 0; alignment(0,2) = 0; 
        alignment(1,0) = 0; alignment(1,1) = 0; alignment(1,2) = -1;
        alignment(2,0) = 0; alignment(2,1) = 1; alignment(2,2) = 0;
        // std::cout << alignment.matrix();
        alignment = alignment.inverse();
        return alignment;//.inverse();
    }

    // ----- get all the tune-able parameters that were used for the implementation, as a string
    std::string getSlamParameterInfo(SlamParameters::SLAMinfo::SLAMinfoPtr info)
    {
        std::string scene_info = "Scene No: " + std::to_string(info->dataset_id_);
        std::string frames = "Frames Processed: " + std::to_string(info->frames_processed_);
        std::string exit_status = (info->process_success_)?" STAM Success":" STAM failed due to correspondence loss";

        std::string baseline = "VO Triangulation Baseline: " + std::to_string(info->visual_odometry_baseline_);

        std::string optimisation = (info->optimisation_thread_on_==true)?"ON":"OFF";

        std::string loop_closure_info = "Loop Closure Constraint Info = (1*"+std::to_string(info->loopclosure_constraint.const1_)+")/(variance*"+std::to_string(info->loopclosure_constraint.const2_)+")";
        std::string odometry_constraint_info = "Odometry Constraint Info = (1*"+std::to_string(info->odometry_constraint.const1_)+")/(variance*"+std::to_string(info->odometry_constraint.const2_)+")";

        std::string matcher = "Repeated Matches Required: min " + std::to_string(info->matcher_min_repetition_)+ ", max " + std::to_string(info->matcher_max_repetition_);

        std::string icp = "Loop Closure Transformation ICP Parameters:-";
        if (info->transform_est_icp.parameters_defined_)
            icp += "\n  --Inlier Threshold: "+std::to_string(info->transform_est_icp.inlier_threshold_)+"; Max Iterations: " + std::to_string(info->transform_est_icp.max_iterations_)+ "; Refinement Sigma: " + std::to_string(info->transform_est_icp.refine_sigma_)+ "; Max Refinement Iterations: " + std::to_string(info->transform_est_icp.refine_max_iterations_);
        else icp += " ICP was not used";

        std::string optical_flow = "LK Optical Flow Parameters:-";
        if (info->lk_parameters.parameters_defined_)
            optical_flow += " Error Tolerence: " + std::to_string(info->lk_parameters.max_correspondence_error_)+"; Min Matches Required: " + std::to_string(info->lk_parameters.min_correspondences_required_) +"; Max Features Used: " + std::to_string(info->lk_parameters.max_correspondence_used_);
        else optical_flow += " Optical Flow was not used";

        std::string fabmap = "FabMap Modifiers:- First Image Frame: " + std::to_string(info->fabmap.first_bow_img_) + "; Frames Skipped: " + std::to_string(info->fabmap.skip_);

        // std::cout << optimisation << std::endl;
        std::string ret_val = "-"+scene_info + "; "  + frames + "; " + exit_status + "\n" + "-"+ baseline + "; Graph Optimization: " +  optimisation + "\n" + "-"+ loop_closure_info + "\n" + "-"+ odometry_constraint_info + "\n" + "-"+ fabmap + "\n" + "-"+ matcher + "\n" + "-"+ icp + "\n" + "-"+ optical_flow;

        return ret_val;
    }

    // ----- ESTIMATING PROJECTION MATRIX USING PNP-RANSAC. ADAPTED AND MODIFIED FROM STAM.
    cv::Mat calcProjMatrix(std::vector<cv::Point2f> points2d, customtype::WorldPtsType points3d, cv::Mat intrinsics, cv::Mat distortion) 
    {


        assert(points2d.size() == points3d.size());

        cv::Mat guess_r, guess_t;
        customtype::WorldPtsType points3d_new = points3d;
        std::vector<cv::Point2f> points2d_new = points2d;

        // std::cout << "end " << points2d.end();
        int max_size = 100;
        if (points2d.size() > 100)
        {
            std::vector<cv::Point2f>::const_iterator first = points2d.end() - max_size;
            std::vector<cv::Point2f>::const_iterator last = points2d.end();
            std::vector<cv::Point2f> new_2d(first, last);
            points2d_new = new_2d;

            std::vector<cv::Point3f>::const_iterator first3 = points3d.end() - max_size;
            std::vector<cv::Point3f>::const_iterator last3 = points3d.end();
            std::vector<cv::Point3f> new_3d(first3, last3);
            points3d_new = new_3d;
        }


        // ---------- Estimate Projection Matrix (rotation and translation vectors) by solving the PnP problem with ransac
        cv::solvePnPRansac(points3d_new, points2d_new, intrinsics, distortion, guess_r, guess_t, false, 100, 5.0, 0.99);
        cv::Mat rvec = guess_r, tvec = guess_t;
        cv::Mat R(3, 3, CV_64FC1);
        cv::Rodrigues(rvec, R);

        cv::Mat Pose(3,4, R.type());
        R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
        cv::Mat projMatrix = Pose;

        return projMatrix;
    }

    // ----- get camera pose from the projection matrix
    customtype::TransformSE3 estimatePoseFromProjection(cv::Mat projmat)
    {
        cv::Mat R1 = projmat(cv::Range::all(), cv::Range(0, 3));
        cv::Mat T1 = projmat(cv::Range::all(), cv::Range(3, 4));

        // ----- Estimating Camera Pose (Pose from projection)
        cv::Mat Pose(3, 4, CV_64FC1);
        cv::Mat pos, R;
        R = R1.inv();
        pos = (-R) * T1;

        // ----- Creating pose matrix 
        R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
        pos.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
        cv::Mat row = (cv::Mat_<double>(1,4) << 0, 0, 0, 1);
        Pose.push_back(row);

        // std::cout << Pose << std::endl;

        // ----- Converting pose from cv::Mat to Eigen::Transform()
        customtype::TransformSE3 eigen_pose;
        cv::cv2eigen(Pose, eigen_pose.matrix());

        // std::cout << eigen_pose.matrix() << std::endl;

        return eigen_pose;

    }

} // namespace slam_utils
} // namespace gSlam