/** @file slam_utils.h (various utility functions required for graph_slam)
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#ifndef __SLAM_UTILS__
#define __SLAM_UTILS__

#include "gslam/data_spot.h"

namespace gSlam
{

namespace slam_utils
{

    template <typename Scalar>
    void getTransformation (Scalar x, Scalar y, Scalar z, Scalar roll, Scalar pitch, Scalar yaw, Eigen::Transform<Scalar, 3, Eigen::Affine> &t)
    {
        Scalar A = cos (yaw),  B = sin (yaw),  C  = cos (pitch), D  = sin (pitch),
                E = cos (roll), F = sin (roll), DE = D*E,         DF = D*F;

        t (0, 0) = A*C;  t (0, 1) = A*DF - B*E;  t (0, 2) = B*F + A*DE;  t (0, 3) = x;
        t (1, 0) = B*C;  t (1, 1) = A*E + B*DF;  t (1, 2) = B*DE - A*F;  t (1, 3) = y;
        t (2, 0) = -D;   t (2, 1) = C*F;         t (2, 2) = C*E;         t (2, 3) = z;
        t (3, 0) = 0;    t (3, 1) = 0;           t (3, 2) = 0;           t (3, 3) = 1;
    }

    customtype::TransformSE3 getTransformation (double x, double y, double z, double roll, double pitch, double yaw);

    customtype::TransformSE3 getTransformation (double x, double y, double z, double qx, double qy, double qz, double qw);

    customtype::TransformSE3 getFrameAligner();
    
    customtype::TransformSE3 getIsmarFrameAligner();

    std::string getSlamParameterInfo(SlamParameters::SLAMinfo::SLAMinfoPtr info);

    cv::Mat calcProjMatrix(std::vector<cv::Point2f> points2d, customtype::WorldPtsType points3d, cv::Mat intrinsics, cv::Mat distortion);

    customtype::TransformSE3 estimatePoseFromProjection(cv::Mat projmat);

} // namespace slam_utils

} // namespace gSlam


#endif // __SLAM_UTILS__