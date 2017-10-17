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

#include "gslam/data_spot.h"
#include "gslam/image_matcher.h"

namespace gSlam 
{

class TransformEstimator
{

public:

    TransformEstimator(){}

    customtype::TransformSE3 estimateTransformUsingOpticalFlow(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                               int& correspondences, int& max_correspondences, double& avg_error, bool& converge_status);

}; // class TransformEstimator

} // namespace gSlam

#endif //__TRANSFORM_ESTIMATOR__