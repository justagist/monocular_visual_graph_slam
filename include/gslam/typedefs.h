#ifndef __TYPEDEFS__
#define __TYPEDEFS__

#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  

namespace gSlam
{

namespace customtype
{
    typedef std::pair<std::vector<cv::Point3d> , std::vector<cv::Point2d>> ProjectionCorrespondences; // pair(vector of 3d points, vector of corresponding 2d image points)

    typedef Eigen::Affine3d TransformSE3; // 4x4 matrix of type double. Affine<Dim>d: the transformation is stored as a (Dim+1)^2 matrix, where the last row is assumed to be [0 ... 0 1].

    typedef std::vector<cv::KeyPoint> p2d_vec; // vector of 2d image points in a frame
    

    typedef pcl::PointXYZ CloudPoint; // A point structure denoting Euclidean xyz coordinates
    typedef pcl::PointCloud<CloudPoint> PointCloud;
    typedef pcl::PointCloud<CloudPoint>::Ptr PointCloudPtr;
} // namespace customtype

} // namespace gSlam





#endif // __TYPEDEFS__