#ifndef __TRANSFORM_ESTIMATOR__
#define __TRANSFORM_ESTIMATOR__

namespace gSlam 
{

class TransformEstimator
{

public:


    TransformEstimator() : ratio_(0.68f), refineF_(true),
        confidence_(0.99), distance_(2.0) // confidence before 0.99, distance before was 3.0
    {
        // SURF is the default feature

        // maxSize – maximum size of the features. The following values are supported: 4, 6, 8, 11, 12, 16, 22, 23, 32, 45, 46, 64, 90, 128. In the case of a different value the result is undefined.
        // responseThreshold – threshold for the approximated laplacian, used to eliminate weak features. The larger it is, the less features will be retrieved
        // lineThresholdProjected – another threshold for the laplacian to eliminate edges
        // lineThresholdBinarized – yet another threshold for the feature size to eliminate edges. The larger the 2nd threshold, the more points you get.


        detector_ = new cv::StarFeatureDetector(32, 10, 18, 18, 20);//new cv::SurfFeatureDetector();
        extractor_ = new cv::SURF(1000, 4, 2, false, true);//new cv::SurfDescriptorExtractor();
    }

    TransformEstimator(float ratio, double confidence, double distance, bool refineF = true) : ratio_(ratio), refineF_(refineF),
        confidence_(confidence), distance_(distance) {
        // SURF is the default feature
        detector_ = new cv::StarFeatureDetector(32, 10, 18, 18, 20);
                //new cv::StarFeatureDetector(32, 10, 18, 18, 20);//new cv::SurfFeatureDetector();
        extractor_ = new cv::SURF(1000, 4, 2, false, true);//new cv::SurfDescriptorExtractor();
        cv::StarFeatureDetector(32, 10, 18, 18, 20);
    }

    custom_type::TransformSE3 estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target, double& variance, int& correspondences, double & prop_match, bool & status_good);

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



    void match(DataSpot3D::DataSpot3DPtr spot_src, DataSpot3D::DataSpot3DPtr spot_target, std::vector<cv::DMatch>& matches);

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





}; // class TransformEstimator

} // namespace gSlam

#endif //__TRANSFORM_ESTIMATOR__