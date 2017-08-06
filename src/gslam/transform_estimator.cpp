#include "gslam/transform_estimator.h"

namespace gSlam
{

    // TransformEstimator::TransformEstimator():
    customtype::TransformSE3 TransformEstimator::estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                   double& variance, int& correspondences, double& prop_matches, bool& converge_status) 
    {
        // CameraParameters src_cam = data_spot_src->getCamParams();
        // CameraParameters tgt_cam = data_spot_target->getCamParams();

        // std::vector<cv::DMatch> matches; // sequence of matches;
        // findMatches(data_spot_src, data_spot_target, matches);
        
        // correspondences = matches.size();

        static bool icp_parameters_defined = false;

        // std::cout << "Repeat status " << repeat_loop_match << std::endl;
        size_t tmp = std::max(data_spot_src->getImagePoints().size(), data_spot_target->getImagePoints().size());
        
        double max_points = (double)std::max(tmp,(size_t)1);

        bool good_match_status = false;
        
        // if( correspondences < 8 )
        // {
        //     status_good = false;
        //     return customtype::TransformSE3();
        // }

        // RELATIVE TRANSFORMATION FROM PROJECTION MATRICES : 
        // customtype::ProjMatType src_proj = data_spot_src->getProjMat();
        // customtype::ProjMatType tgt_proj = data_spot_target->getProjMat();        
        // customtype::TransformSE3 out_transform = slam_utils::estimateRelativeTransformBtwnProjections(src_proj,tgt_proj);
        // status_good = true; // TEMPORARY
        // variance = 1; // TEMPORARY

        variance = 1;
        customtype::WorldPtsType src_wrldpts;
        customtype::WorldPtsType tgt_wrldpts;

        // if (repeat_match_counter_>max_repeat_match_counter_)
        //     repeat_match_counter_ = 0;

        // if (repeat_loop_match)
        //     repeat_match_counter_++;
        // else repeat_match_counter_ = 0;





        //// this method should be used only if the images are distorted -----------------

        // spot_matcher_.findMatchingWorldpoints(data_spot_src, data_spot_target, src_wrldpts, tgt_wrldpts, good_match_status);

        // // ----------------------------------------------------------------------------------


        // using undistorted image
        spot_matcher_.findMatchingWorldpoints(data_spot_src->getImageColor(), data_spot_target->getImageColor(), data_spot_src->getImagePoints(), data_spot_target->getImagePoints(), data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints(), src_wrldpts, tgt_wrldpts, good_match_status);//, data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints())

        // REMOVE ++++++++++++++++++++++++++_+===================
        // return customtype::TransformSE3();
        // =++++++++++++++++++++=+++++++++++++==========++++++=========

        // std::cout << data_spot_target->getImagePoints().size() << " checking " << std::endl;
        // std::cout << data_spot_src->getImagePoints().size() << " checking " << std::endl;
        // std::cout << "here !!" << std::endl;
        // if (src_wrldpts.size() < 0.5*data_spot_src->getWorldPoints().size() or src_wrldpts.size() < 0.5*data_spot_target->getWorldPoints().size())
        assert (src_wrldpts.size() == tgt_wrldpts.size());
        correspondences = src_wrldpts.size();
        customtype::TransformSE3 relative_transformation;
        if (correspondences > 0 && good_match_status)
        {
            // src_wrldpts = data_spot_src->getWorldPoints();
            // tgt_wrldpts = data_spot_target->getWorldPoints();
        
            customtype::PointCloudPtr src_cloud(new customtype::PointCloud());
            customtype::PointCloudPtr tgt_cloud(new customtype::PointCloud());

            src_cloud = slam_utils::convert3dPointsToCloud(src_wrldpts);
            tgt_cloud = slam_utils::convert3dPointsToCloud(tgt_wrldpts);


            // std::cout << "tgt" << tgt_cloud << std::endl;
            // bool converge_status;

            // relative_transformation = slam_utils::icp(src_cloud, tgt_cloud, 0.1, 50, &converge_status, &variance, &correspondences);
            std::vector<int> inliers;

            double inlierThreshold = 10;
            int iterations = 10000;
            double refineModelSigma = 3;
            int refineModelIterations = 100;

            relative_transformation = slam_utils::transformFromXYZCorrespondences(src_cloud, tgt_cloud, inlierThreshold, iterations, true, refineModelSigma, refineModelIterations , &inliers, &variance, converge_status);

            if (!icp_parameters_defined)
            {
                SlamParameters::info->transform_est_icp.inlier_threshold_ = inlierThreshold;
                SlamParameters::info->transform_est_icp.max_iterations_ = iterations;
                SlamParameters::info->transform_est_icp.refine_sigma_ = refineModelSigma;
                SlamParameters::info->transform_est_icp.refine_max_iterations_ = refineModelIterations;
                SlamParameters::info->transform_est_icp.parameters_defined_ = true;
                icp_parameters_defined = true;
            }
            // converge_status = true;

            prop_matches = double(correspondences)/max_points;
            std::cout << "converge_status: "  << std::boolalpha << converge_status << std::noboolalpha << " variance: " << variance << " correspondences: " << correspondences << " prop_matches: " << prop_matches << std::endl;
            // cv::waitKey(0);
            if (converge_status)
            {
                // std::cout <<"in transform_estimator " << relative_transformation.matrix() << std::endl;
                return relative_transformation;
            }
            else
            {
                std::cout << " ICP Transformation failed. Sending Identity Transform.\n";
                return customtype::TransformSE3::Identity();
            }

            // std::cout << relative_transformation.matrix() << std::endl;
        }
        else
        {
            converge_status = false;
            if (!good_match_status)
                std::cout << "False Positive Loop Closure Ignored" << std::endl;
            if (correspondences <= 0) 
                std::cout << "Too few correspondences to estimate PointCloud Transformation" << std::endl;
            std::cout << "converge_status: "  << std::boolalpha << converge_status << std::noboolalpha << "; variance: " << variance << " correspondences: " << correspondences << std::endl; 
            // std::cout << "returning default TransformSE3 value " << customtype::TransformSE3().matrix() << std::endl;
            // cv::waitKey(0);
            // customtype::TransformSE3 test_mat = customtype::TransformSE3::Identity();
            return customtype::TransformSE3::Identity();
        }

        // if(correspondences == 0)
        // {
        //     relative_transformation = data_spot_src->getPose().inverse()*data_spot_target->getPose();
        // }


    }

    // TODO: estimate relative transform as follows: obtain the pose at the 'loop closure image' using tracking similar to STAM, by calculating optical flow from the current image, and then calculating its projection matrix. The relative pose between this pose and the 'actual' pose of the loop image should be the loop closure constraint.

    // ******** Tracking from tgt --> src **********
    // tgt: current image
    // src: loop closure match image
    customtype::TransformSE3 TransformEstimator::estimateTransformUsingOpticalFlow(DataSpot3D::DataSpot3DPtr data_spot_src, 
                                                                                   DataSpot3D::DataSpot3DPtr data_spot_target, 
                                                                                   int& correspondences, int& max_correspondences, 
                                                                                   bool& converge_status)
    {

        static bool opt_flow_parameters_defined = false;

        // TODO: IF FAILS, TRY CREATING A OPTICAL FLOW PYRAMID OF THE FIRST IMAGE USING BUILDOPTICALFLOWPYRAMID FUNCTION AND PASS TO THE CALCOPTICALFLOWPYRLK FUNCTION INSTEAD OF THE INPUT IMAGE 1

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
        int max_features = 1000;
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
        int opt_flow_min_match_reqd = 50; // was 50;

        // ------- recording parameters to SlamParameters 
        if (!opt_flow_parameters_defined)
        {
            SlamParameters::info->lk_parameters.max_correspondence_error_ = opt_flow_err_tol;
            SlamParameters::info->lk_parameters.min_correspondences_required_ = opt_flow_min_match_reqd;
            SlamParameters::info->lk_parameters.parameters_defined_ = true;
            opt_flow_parameters_defined = true;
        }

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
            }
        }

        // -------------------------------------------------------------------

        std::cout << tgt_points_new.size() << " = tgt_pts size; " << filtered_src.size() << " = filtered_src size" << std::endl;

        // ------------------ Estimating Projection matrix and pose if enough features are matched across the loop closure
        if (filtered_src.size() >= opt_flow_min_match_reqd || filtered_src.size()>0.5*tgt_points_new.size())
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
            // ----- Use frame transform to align camera frame to ardrone body frame =================== requiredonly while using the drone datasets
            pose_estimate = pose_estimate * SlamParameters::pose_aligner_;
            
            std::cout << "Estimated Pose in Loop Closure Frame: \n" << pose_estimate.matrix() << std::endl;
            // std::cout << "Translation vector: " << pose_estimate.translation().z() << " " << pose_estimate(3,3) << std::endl << pose_estimate(2,3) << std::endl; 

            correspondences = filtered_src.size();
            max_correspondences = tgt_points_new.size();
            // variance = 1/prop_matches;
            converge_status = true;

            std::cout << "data_spot_src actual pose \n " << data_spot_src->getPose().matrix() << std::endl;
            std::cout << "data_spot_target actual pose \n " << data_spot_target->getPose().matrix() << std::endl;
            customtype::TransformSE3 inv =  data_spot_target->getPose().inverse();
            std::cout << inv.matrix() << std::endl;

            // ----------- Relative transform estimate if the projection estimate is obtained
            customtype::TransformSE3 relative_transformation = data_spot_target->getPose().inverse()*pose_estimate;
            std::cout << "Estimated Relative Transform: \n" << relative_transformation.matrix() << std::endl;

            // ========================== DEBUG
            bool show_optflow_matches = true;
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
                // cv::waitKey(0);
            }
            // ==========================

            return relative_transformation;

        }

        // ====================

        // customtype::TransformSE3 relative_transformation;



        return customtype::TransformSE3();


    }

    

} // namespace gSlam


    
