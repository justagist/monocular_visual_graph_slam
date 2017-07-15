#ifndef __TRANSFORM_ESTIMATOR__
#define __TRANSFORM_ESTIMATOR__

// #include "typedefs.h"
#include "gslam/data_spot.h"
#include "gslam/slam_utils.h"
// #include "gslam/dataspot_matcher.h"

namespace gSlam 
{

class TransformEstimator
{

public:

    customtype::TransformSE3 estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target, double& variance, int& correspondences, double & prop_match, bool & status_good);

private:
    slam_utils::DataSpotMatcher spot_matcher_;

}; // class TransformEstimator

} // namespace gSlam

#endif //__TRANSFORM_ESTIMATOR__