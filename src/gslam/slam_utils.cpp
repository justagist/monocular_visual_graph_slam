#include "gslam/slam_utils.h"

namespace gSlam
{

namespace slam_utils
{
    // gives transformation of image 2 to image 1 (tgt_prj to src_prj)
    customtype::TransformSE3 estimateRelativeTransformBtwnImages(customtype::ProjMatType src_prj, customtype::ProjMatType tgt_prj)
    {
        Eigen::Matrix3d R_src = src_prj.block(0,0,3,3);
        Eigen::Matrix3d R_tgt = tgt_prj.block(0,0,3,3);

        Eigen::Matrix<double,3,1> t_src = src_prj.col(3);
        Eigen::Matrix<double,3,1> t_tgt = tgt_prj.col(3);

        Eigen::Matrix3d R_out = (R_src.inverse())*R_tgt;
        Eigen::Matrix<double,3,1> t_out = R_src.inverse()*(t_tgt - t_src);

        customtype::TransformSE3 out_transform;
        Eigen::Matrix<double,1,4> vect;
        vect << 0,0,0,1;
        out_transform.matrix() << R_out, t_out, vect;
        
        return out_transform;
    }

} // namespace slam_utils
} // namespace gSlam