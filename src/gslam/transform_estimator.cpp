#include "gslam/transform_estimator.h"

namespace gSlam
{
    customtype::TransformSE3 TransformEstimator::estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                   double& variance, int& correspondences, double& prop_matches, bool& converge_status) 
    {
        // CameraParameters src_cam = data_spot_src->getCamParams();
        // CameraParameters tgt_cam = data_spot_target->getCamParams();

        // std::vector<cv::DMatch> matches; // sequence of matches;
        // findMatches(data_spot_src, data_spot_target, matches);
        
        // correspondences = matches.size();
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
            relative_transformation = slam_utils::transformFromXYZCorrespondences(src_cloud, tgt_cloud, 0.05, 100, true, 3.0, 5, &inliers, &variance);
            converge_status = true;
            prop_matches = double(correspondences)/max_points;
            std::cout << "converge_status: "  << std::boolalpha << converge_status << std::noboolalpha << " variance: " << variance << " correspondences: " << correspondences << " prop_matches: " << prop_matches << std::endl;
            cv::waitKey(0);
            return relative_transformation;

            // std::cout << relative_transformation.matrix() << std::endl;
        }
        else
        {
            converge_status = false;
            if (!good_match_status)
                std::cout << "False Positive Loop Closure Ignored" << std::endl;
            if (correspondences <= 0) 
                std::cout << "Too few correspondences to estimate PointCloud Transformation" << std::endl;
            std::cout << "converge_status: "  << std::boolalpha << converge_status << std::noboolalpha << " variance: " << variance << " correspondences: " << correspondences << std::endl; 
            // std::cout << "returning default TransformSE3 value " << customtype::TransformSE3().matrix() << std::endl;
            // cv::waitKey(0);
            return customtype::TransformSE3().Identity();
        }

        // if(correspondences == 0)
        // {
        //     relative_transformation = data_spot_src->getPose().inverse()*data_spot_target->getPose();
        // }


    }

    

} // namespace gSlam


    
