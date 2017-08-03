#ifndef __SLAM_UTILS__
#define __SLAM_UTILS__

// #include "gslam/typedefs.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>
#include "gslam/data_spot.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>


namespace gSlam
{

namespace slam_utils
{
    
    customtype::TransformSE3 estimateRelativeTransformBtwnProjections(customtype::ProjMatType src_prj, customtype::ProjMatType tgt_prj);

    // Eigen::Matrix4d icp2D(const customtype::PointCloudPtr & cloud_source,
    //                       const customtype::PointCloudPtr & cloud_target, //TODO: Should be ConstPtr
    //                       double maxCorrespondenceDistance,
    //                       int maximumIterations,
    //                       bool * hasConvergedOut,
    //                       double * variance,
    //                       int * correspondencesOut){};

    void getTranslationAndEulerAngles (const customtype::TransformSE3& t,
                                   float& x, float& y, float& z,
                                   float& roll, float& pitch, float& yaw);

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

    void computeVariance(const customtype::PointCloudPtr & cloud_source,
                       const customtype::PointCloudPtr & cloud_target,
                     const customtype::TransformSE3& rel_transform, // rel_transform:how much the sensor moved w.r.t to world
                     double maxCorrespondenceDistance,
                       bool * hasConvergedOut,
                       double * variance,
                       int * correspondencesOut);

    customtype::PointCloudPtr getICPReadyCloud(
            const customtype::PointCloudPtr cloud_in,
            float voxel,
            int samples,
            const customtype::TransformSE3 & transform);

    customtype::PointCloudPtr voxelize(
            const customtype::PointCloudPtr & cloud,
            float voxelSize);

    customtype::PointCloudPtr sampling(
            const customtype::PointCloudPtr & cloud, int samples);

    Eigen::Matrix4d transformFromXYZCorrespondences(const customtype::PointCloudPtr & cloud1,
                                                    const customtype::PointCloudPtr & cloud2,
                                                    double inlierThreshold,
                                                    int iterations,
                                                    bool refineModel,
                                                    double refineModelSigma,
                                                    int refineModelIterations,
                                                    std::vector<int> * inliersOut,
                                                    double * varianceOut,
                                                    bool& got_transform);

    customtype::PointCloudPtr convert3dPointsToCloud(customtype::WorldPtsType wrldpts);

    Eigen::Matrix4d icp(const customtype::PointCloudPtr & cloud_source, //TODO: Should be ConstPtr
                        const customtype::PointCloudPtr & cloud_target,
                        double maxCorrespondenceDistance,
                        int maximumIterations,
                        bool * hasConvergedOut,
                        double * variance,
                        int * correspondencesOut);

    std::string getSlamParameterInfo(SlamParameters::SLAMinfo::SLAMinfoPtr info);

} // namespace slam_utils

} // namespace gSlam


#endif // __SLAM_UTILS__