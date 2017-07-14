#ifndef __TYPEDEFS__
#define __TYPEDEFS__

#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
// #include <boost/shared_ptr.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include "STAM.h"
// #include <iostream>

namespace gSlam
{

namespace customtype
{
    typedef std::pair<std::vector<cv::Point3d> , std::vector<cv::Point2d>> ProjectionCorrespondences; // pair(vector of 3d points, vector of corresponding 2d image points)

    typedef Eigen::Affine3d TransformSE3; // 4x4 matrix of type double. Affine<Dim>d: the transformation is stored as a (Dim+1)^2 matrix, where the last row is assumed to be [0 ... 0 1].


    typedef unsigned int Identifier;

    typedef pcl::PointXYZ CloudPoint; // A point structure denoting Euclidean xyz coordinates
    typedef pcl::PointCloud<CloudPoint> PointCloud;
    typedef pcl::PointCloud<CloudPoint>::Ptr PointCloudPtr;

    typedef std::mutex Mutex;
    typedef std::unique_lock<Mutex> Lock; // unique_lock enables choosing std::try_to_lock_t, std::adopt_lock_t or std::defer_lock_t
    typedef std::condition_variable CondVar;

    typedef Eigen::Matrix<double,6,6> InformationMatrix3D;

    typedef Eigen::Matrix<double,3,4> ProjMatType; // projection matrix

    typedef std::vector<cv::KeyPoint> KeyPoints; // vector of 2d image points in a frame
    typedef std::vector<cv::Point3f> WorldPtsType;
    // typedef std::vector<cv::KeyPoint
} // namespace customtype

} // namespace gSlam





#endif // __TYPEDEFS__