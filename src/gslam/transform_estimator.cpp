#include "gslam/transform_estimator.h"

namespace gSlam
{
    customtype::TransformSE3 TransformEstimator::estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                   double& variance, int& correspondences, double& prop_matches, bool& status_good) 
    {
        // CameraParameters src_cam = data_spot_src->getCamParams();
        // CameraParameters tgt_cam = data_spot_target->getCamParams();

        // std::vector<cv::DMatch> matches; // sequence of matches;
        // findMatches(data_spot_src, data_spot_target, matches);
        
        // correspondences = matches.size();
        // size_t tmp = std::max(data_spot_src->getKeyPoints().size(), data_spot_target->getKeyPoints().size());
        
        // double max_points = (double)std::max(tmp,(size_t)1);
        // prop_matches = double(correspondences)/max_points;
        
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


        customtype::WorldPtsType src_wrldpts;
        customtype::WorldPtsType tgt_wrldpts;
        // customtype::

        spot_matcher_.findMatchingWorldpoints(data_spot_src->getImageColor(), data_spot_target->getImageColor(), data_spot_src->getKeyPoints(), data_spot_target->getKeyPoints(), data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints(), src_wrldpts, tgt_wrldpts);//, data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints())

        customtype::PointCloudPtr src_cloud(new customtype::PointCloud());
        customtype::PointCloudPtr tgt_cloud(new customtype::PointCloud());

        src_cloud = slam_utils::convert3dPointsToCloud(src_wrldpts);
        tgt_cloud = slam_utils::convert3dPointsToCloud(tgt_wrldpts);





    }

    

} // namespace gSlam


    
