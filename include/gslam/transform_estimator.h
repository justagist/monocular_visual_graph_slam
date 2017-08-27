/** @file transform_estimator.h (class for estimating relative transformation of two poses between whom loop closure is possible. Also checks if loop closure is valid using 'optical-flow-check')
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#ifndef __TRANSFORM_ESTIMATOR__
#define __TRANSFORM_ESTIMATOR__

// #include "typedefs.h"
#include "gslam/data_spot.h"
#include "gslam/image_matcher.h"
#include "gslam/slam_utils.h"
// #include "gslam/dataspot_matcher.h"

namespace gSlam 
{

class TransformEstimator
{

public:

    TransformEstimator(){}

    customtype::TransformSE3 estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target, double& variance, int& correspondences, double & prop_match, bool & converge_status);

    customtype::TransformSE3 estimateTransformUsingOpticalFlow(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                               int& correspondences, int& max_correspondences, double& avg_error, bool& converge_status);

private:
    slam_utils::ImageMatcher spot_matcher_;
    // float opt_flow_err_tol_;
    // int opt_flow_min_match_reqd_;
    // cv::Mat::intrinsics_, distortion_;
    // int repeat_match_counter_, max_repeat_match_counter_;

}; // class TransformEstimator

} // namespace gSlam

#endif //__TRANSFORM_ESTIMATOR__