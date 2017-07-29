#ifndef __SLAM_UTILS__
#define __SLAM_UTILS__

#include "gslam/typedefs.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>
#include "gslam/data_spot.h"


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

    Eigen::Matrix4d transformFromXYZCorrespondences(const customtype::PointCloudPtr & cloud1,
                                                    const customtype::PointCloudPtr & cloud2,
                                                    double inlierThreshold,
                                                    int iterations,
                                                    bool refineModel,
                                                    double refineModelSigma,
                                                    int refineModelIterations,
                                                    std::vector<int> * inliersOut,
                                                    double * varianceOut);

    customtype::PointCloudPtr convert3dPointsToCloud(customtype::WorldPtsType wrldpts);

    Eigen::Matrix4d icp(const customtype::PointCloudPtr & cloud_source, //TODO: Should be ConstPtr
                        const customtype::PointCloudPtr & cloud_target,
                        double maxCorrespondenceDistance,
                        int maximumIterations,
                        bool * hasConvergedOut,
                        double * variance,
                        int * correspondencesOut);



    // DATASPOT MATCHER CLASS

    class DataSpotMatcher {
    public:

        DataSpotMatcher() : ratio_(0.75f), refineF_(true),
            confidence_(0.99), distance_(2.0) { // confidence before 0.99, distance before was 3.0
            // SURF is the default feature

            // maxSize – maximum size of the features. The following values are supported: 4, 6, 8, 11, 12, 16, 22, 23, 32, 45, 46, 64, 90, 128. In the case of a different value the result is undefined.
            // responseThreshold – threshold for the approximated laplacian, used to eliminate weak features. The larger it is, the less features will be retrieved
            // lineThresholdProjected – another threshold for the laplacian to eliminate edges
            // lineThresholdBinarized – yet another threshold for the feature size to eliminate edges. The larger the 2nd threshold, the more points you get.


            detector_ = new cv::StarFeatureDetector(32, 10, 18, 18, 20);//new cv::SurfFeatureDetector();
            // extractor_ = new cv::SURF(1000, 4, 7, true, false);//new cv::SurfDescriptorExtractor();
            // extractor_ = new cv::SIFT(0,3,0.04,10,1.6);
            extractor_ = new cv::SurfDescriptorExtractor(1000, 4, 2, true, true);
        }

        DataSpotMatcher(float ratio, double confidence, double distance, bool refineF = true) : ratio_(ratio), refineF_(refineF),
            confidence_(confidence), distance_(distance) {
            // SURF is the default feature
            detector_ = new cv::StarFeatureDetector(32, 10, 18, 18, 20);
                    //new cv::StarFeatureDetector(32, 10, 18, 18, 20);//new cv::SurfFeatureDetector();
            extractor_ = new cv::SURF(1000, 4, 2, true, false);//new cv::SurfDescriptorExtractor();
            // extractor_ = new cv::SIFT();
            cv::StarFeatureDetector(32, 10, 18, 18, 20);
        }


        // Set the feature detector
        void setFeatureDetector(
                cv::Ptr<cv::FeatureDetector>& detect) {
            detector_ = detect;
        }

        // Set the descriptor extractor
        void setDescriptorExtractor(
                cv::Ptr<cv::DescriptorExtractor>& desc) {
            extractor_ = desc;
        }


        void findMatchingWorldpoints(cv::Mat image1, cv::Mat image2, 
                                     customtype::KeyPoints imgpts1,
                                     customtype::KeyPoints imgpts2, 
                                     customtype::WorldPtsType wrldpts1,
                                     customtype::WorldPtsType wrldpts2,
                                     customtype::WorldPtsType& out_1,
                                     customtype::WorldPtsType& out_2);

        void findMatches(DataSpot3D::DataSpot3DPtr spot_src, DataSpot3D::DataSpot3DPtr spot_target, std::vector<cv::DMatch>& matches);

    private:

        // Match feature points using symmetry test and RANSAC
        // returns fundemental matrix
        cv::Mat findMatches(cv::Mat& image1,
                      cv::Mat& image2, // input images
                      // output matches and keypoints
                      std::vector<cv::DMatch>& matches,
                      std::vector<cv::KeyPoint>& keypoints1,
                      std::vector<cv::KeyPoint>& keypoints2);


        // Clear matches for which NN ratio is > than threshold
        // return the number of removed points
        // (corresponding entries being cleared,
        // i.e. size will be 0)
        int ratioTest(std::vector<std::vector< cv::DMatch> >
                      &matches);

        // Insert symmetrical matches in symMatches vector
        void symmetryTest(
                const std::vector<std::vector< cv::DMatch> >& matches1,
                const std::vector<std::vector< cv::DMatch> >& matches2,
                std::vector<cv::DMatch>& symMatches);
        void symmetryTest(
                const std::vector< cv::DMatch>& matches1,
                const std::vector< cv::DMatch>& matches2,
                std::vector<cv::DMatch>& symMatches);

        // Identify good matches using RANSAC
        // Return fundemental matrix
        cv::Mat ransacTest(
                const std::vector<cv::DMatch>& matches,
                const std::vector<cv::KeyPoint>& keypoints1,
                const std::vector<cv::KeyPoint>& keypoints2,
                std::vector<cv::DMatch>& outMatches);

        // pointer to the feature point detector object
        cv::Ptr<cv::FeatureDetector> detector_;
        // pointer to the feature descriptor extractor object
        cv::Ptr<cv::DescriptorExtractor> extractor_;
        float ratio_; // max ratio between 1st and 2nd NN
        bool refineF_; // if true will refine the F matrix
        double distance_; // min distance to epipolar
        double confidence_; // confidence level (probability)

    }; // class dataspotmatcher

} // namespace slam_utils

} // namespace gSlam


#endif // __SLAM_UTILS__