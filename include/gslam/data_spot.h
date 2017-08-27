/** @file dataspot.h (Creates an object storing all data received in a particular frame/iteration (acts like a node of the graph))
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#ifndef __DATASPOT__
#define __DATASPOT__

#include "gslam/typedefs.h"
namespace gSlam
{

template <typename TransformType, typename InfMatrixType>
class DataLink{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::shared_ptr< DataLink<TransformType, InfMatrixType> > DataLinkPtr;
    typedef std::multimap< customtype::Identifier, DataLinkPtr > Links;

    enum { OdomConstraint, LoopClosureConstraint, Unspecified };

    DataLink(){};
    DataLink(customtype::Identifier from_id, customtype::Identifier to_id,const TransformType& transform,const InfMatrixType& inf_matrix);

    customtype::Identifier from_id_;
    customtype::Identifier to_id_;
    TransformType transform_;
    InfMatrixType inf_matrix_;

    bool active;

    int type;

};

typedef DataLink<customtype::TransformSE3, customtype::InformationMatrix3D> DataLink3D;


class CameraParameters{
public:

    Eigen::Matrix3d intrinsicsMat_;
    cv::Mat intrinsics_;
    cv::Mat distortion_;

    CameraParameters(const cv::Mat& intrinsics, const cv::Mat& distortion)
    {
        intrinsics_ = intrinsics; 
        distortion_ = distortion;
        cv::cv2eigen(intrinsics,intrinsicsMat_);
    }


}; // class CameraParameters

class DataSpot3D
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::shared_ptr<DataSpot3D> DataSpot3DPtr;
    typedef std::map<customtype::Identifier,DataSpot3DPtr> DataSpotMap;

    DataSpot3D();

    DataSpot3D(const customtype::TransformSE3& pose, const CameraParameters& camParams, const cv::Mat& image_color,const customtype::WorldPtsType& world_pts, const customtype::KeyPoints& img_pts); // Timestamp?

    customtype::Identifier getId() const { return id_; }
    void setId(customtype::Identifier id) { id_ = id; }
    customtype::TransformSE3 getPose() const { return pose_; }

    void addLink(customtype::Identifier to_id, customtype::TransformSE3 rel_transf, customtype::InformationMatrix3D inf_matrix = customtype::InformationMatrix3D::Identity()) 
    {
        links_.insert(std::make_pair(id_, DataLink3D::DataLinkPtr(new DataLink3D(id_, to_id, rel_transf, inf_matrix))));
    }

    void addLink(const DataLink3D::DataLinkPtr& link) 
    {
        links_.insert(std::make_pair(id_, link));
    }

    void setPose(const customtype::TransformSE3& pose)
    {
        pose_ = pose;
    }

    DataLink3D::Links& getLinks()
    {
        return links_;
    }

    customtype::KeyPoints& getKeyPoints()
    {
        return keypoints_;
    }

    customtype::WorldPtsType& getWorldPoints()
    {
        return world_pts_;
    }

    customtype::KeyPoints& getImagePoints()
    {
        return image_pts_;
    }

    cv::Mat& getImageColor()
    {
        return image_color_;
    }

    CameraParameters getCamParams() 
    {
        return camParams_;
    }

private:

    static customtype::Identifier next_id_s;

    bool added_to_graph_;
    customtype::Identifier id_;
    customtype::TransformSE3 pose_;

    DataLink3D::Links links_;

    CameraParameters camParams_;
    std::multimap<customtype::Identifier, customtype::CloudPoint> keypoints3D_;
    cv::Mat descriptors_;
    cv::Mat image_color_;

    // customtype::ImgPtsType img_pts_;
    customtype::WorldPtsType world_pts_; // 3d world points tracked by STAM in each frame
    customtype::KeyPoints image_pts_; // corresponding 2d points from STAM

    customtype::KeyPoints keypoints_; // keypoints for fabmap
}; // class DataSpot3D

} // namespace gSlam

#endif // __DATASPOT__