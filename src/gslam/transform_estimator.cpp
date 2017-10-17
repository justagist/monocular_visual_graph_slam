/** @file transform_estimator.cpp (class for estimating relative transformation of two poses between whom loop closure is possible. Also checks if loop closure is valid using 'optical-flow-check')
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#include "gslam/transform_estimator.h"

namespace gSlam
{

    // ----- Estimates relative transform as follows: obtain the pose at the 'loop closure image' using tracking similar to STAM, by calculating optical flow from the current image, and then calculating its projection matrix. The relative pose between this pose and the tgt pose should be the constraint

    // ******** Tracking from tgt --> src **********
    // tgt: current image
    // src: loop closure match image
    customtype::TransformSE3 TransformEstimator::estimateTransformUsingOpticalFlow(DataSpot3D::DataSpot3DPtr data_spot_src, 
                                                                                   DataSpot3D::DataSpot3DPtr data_spot_target, 
                                                                                   int& correspondences, int& max_correspondences, 
                                                                                   double& avg_error, bool& converge_status)
    {
        auto t1 = customtype::Clock::now();
        static bool opt_flow_parameters_defined = false;

        converge_status = false;

        // ---------------------- calculate optical flow from tgt img to src img, and obtain filtered keypoints in the src img
        cv::Mat tgt_gray, src_gray;
        std::vector<cv::Point2f> tgt_points, tgt_points_new;
        cv::KeyPoint::convert(data_spot_target->getImagePoints(), tgt_points);
        cv::cvtColor(data_spot_target->getImageColor(), tgt_gray, CV_BGR2GRAY);

        std::vector<cv::Point2f> src_points;
        cv::cvtColor(data_spot_src->getImageColor(), src_gray, CV_BGR2GRAY);
        std::vector<uchar> status;
        cv::Mat errors;

        // imshow("current_frame", tgt_gray);
        // cv::waitKey(1);
        int max_features = 2000; // was 1000
        if (tgt_points.size() > max_features)
        {
            std::vector<cv::Point2f>::const_iterator first = tgt_points.end() - max_features;
            std::vector<cv::Point2f>::const_iterator last = tgt_points.end();
            std::vector<cv::Point2f> new_pts(first, last);
            tgt_points_new = new_pts;
        }
        else tgt_points_new = tgt_points;

        cv::calcOpticalFlowPyrLK(tgt_gray, src_gray, tgt_points_new, src_points, status, errors);

        // ------- Defining optical flow thresholds
        float opt_flow_err_tol = 12.0; // was 12.0
        int opt_flow_min_match_reqd = 150; // was 50 with max_features 1000;

        // ------- recording parameters to SlamParameters 
        if (!opt_flow_parameters_defined)
        {
            SlamParameters::info->lk_parameters.max_correspondence_error_ = opt_flow_err_tol;
            SlamParameters::info->lk_parameters.min_correspondences_required_ = opt_flow_min_match_reqd;
            SlamParameters::info->lk_parameters.parameters_defined_ = true;
            SlamParameters::info->lk_parameters.max_correspondence_used_ = max_features;
            opt_flow_parameters_defined = true;
        }

        // ------- Calculating average error for variance estimation
        double accumulated_error = 0;

        // ------- Removing features that were not matched
        std::vector<cv::Point2f> filtered_src, filtered_tgt;
        customtype::WorldPtsType filtered_src_3D;
        for (int i = 0; i < tgt_points_new.size(); i++) 
        {
            if (status[i] != 0 && errors.at<float>(i) < opt_flow_err_tol)   // klt ok!
            {

                filtered_src.push_back(src_points[i]);
                filtered_tgt.push_back(tgt_points[i]);
                filtered_src_3D.push_back(data_spot_target->getWorldPoints()[i]);
                accumulated_error += errors.at<float>(i);
            }
        }

        avg_error = accumulated_error/filtered_src.size();
        // -------------------------------------------------------------------

        std::cout << tgt_points_new.size() << " = tgt_pts size; " << filtered_src.size() << " = filtered_src size; " << avg_error << " = average error"<<std::endl;
        auto t2 = customtype::Clock::now();
        std::cout << "Optical flow check Duration " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds" << std::endl;
        // std::cin.get();
        
        // ------------------ Estimating Projection matrix and pose if enough features are matched across the loop closure
        if (filtered_src.size() >= opt_flow_min_match_reqd || filtered_src.size()>0.7*tgt_points_new.size())// && tgt_points_new.size() > 700)//|| filtered_src.size()>0.5*tgt_points_new.size())
        {

            cv::Mat projMat = slam_utils::calcProjMatrix(filtered_src, filtered_src_3D, data_spot_src->getCamParams().intrinsics_, data_spot_src->getCamParams().distortion_);
            std::cout << "Projection Matrix:\n" << projMat << std::endl;

            // -------------- Testing if the returned projection matrix is Identity, i.e. estimation failed 
            cv::Mat eye = cv::Mat::eye(3,4, CV_64FC1);
            cv::Mat dst;
            cv::bitwise_xor(projMat, eye, dst);        
            if(cv::countNonZero(dst) <= 0)
            {
                std::cout << "^^^^^^^^^^^^^ Projection Matrix could not be estimated. Ignoring Loop Closure ^^^^^^^^^^" <<std::endl;
                return customtype::TransformSE3();
            }
            // ----------------------

            customtype::TransformSE3 pose_estimate = slam_utils::estimatePoseFromProjection(projMat);
            std::cout << "Estimated Pose in Loop Closure Frame before alignment: \n" << pose_estimate.matrix() << std::endl;
            // ----- Use frame transform to align camera frame to ardrone body frame =================== requiredonly while using the drone datasets
            pose_estimate = pose_estimate * SlamParameters::pose_aligner_; // not using gives better result -- ???

            std::cout << "Estimated Pose in Loop Closure Frame after frame alignment: \n" << pose_estimate.matrix() << std::endl;
            // std::cout << "Translation vector: " << pose_estimate.translation().z() << " " << pose_estimate(3,3) << std::endl << pose_estimate(2,3) << std::endl; 

            correspondences = filtered_src.size();
            max_correspondences = tgt_points_new.size();
            // variance = 1/prop_matches;
            converge_status = true;

            std::cout << "data_spot_src actual pose \n " << data_spot_src->getPose().matrix() << std::endl;
            std::cout << "data_spot_target actual pose \n " << data_spot_target->getPose().matrix() << std::endl;
            // customtype::TransformSE3 inv =  data_spot_target->getPose().inverse();
            // std::cout << inv.matrix() << std::endl;

            // // ----------- Relative transform estimate if the projection estimate is obtained
            customtype::TransformSE3 relative_transformation = data_spot_target->getPose().inverse()*pose_estimate;
            customtype::TransformSE3 pose_correct = customtype::TransformSE3::Identity();
            pose_correct.translation() = relative_transformation.translation();

            // ============================================================================================
            // customtype::TransformSE3 inv = pose_estimate.inverse();
            // customtype::TransformSE3 relative_transformation = pose_estimate.inverse()*data_spot_target->getPose();
            std::cout << "Estimated Relative Transform: \n" << relative_transformation.matrix() << std::endl;

            // ========================== DEBUG
            bool show_optflow_matches = false;
            if (show_optflow_matches)
            {
                customtype::KeyPoints kp1, kp2;
                cv::Mat out1, out2;
                cv::KeyPoint::convert(filtered_tgt, kp1);
                cv::KeyPoint::convert(filtered_src, kp2);
                cv::drawKeypoints(data_spot_target->getImageColor(), kp1, out1);
                cv::drawKeypoints(data_spot_src->getImageColor(), kp2, out2);
                cv::imshow("matching_tgt", out1);
                cv::imshow("matching_src", out2);
                cv::waitKey(0);
            }
            // ==========================

            return relative_transformation;
            // return pose_correct;
        }

        return customtype::TransformSE3();
    }

    

} // namespace gSlam


    
