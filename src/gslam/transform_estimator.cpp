#include "gslam/transform_estimator.h"

namespace gSlam
{
    customtype::TransformSE3 TransformEstimator::estimateTransform(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                   double& variance, int& correspondences, double& prop_matches, bool& status_good) 
    {
        // CameraParameters src_cam = data_spot_src->getCamParams();
        // CameraParameters tgt_cam = data_spot_target->getCamParams();

        // std::vector<cv::DMatch> matches; // sequence of matches;
        // findMatches(data_spot_src, data_spot_target, matches);
        
        // correspondences = matches.size();
        // size_t tmp = std::max(data_spot_src->getKeyPoints().size(), data_spot_target->getKeyPoints().size());
        
        // double max_points = (double)std::max(tmp,(size_t)1);
        // prop_matches = double(correspondences)/max_points;
        
        // if( correspondences < 8 )
        // {
        //     status_good = false;
        //     return customtype::TransformSE3();
        // }


        customtype::ProjMatType src_proj = data_spot_src->getProjMat();
        customtype::ProjMatType tgt_proj = data_spot_target->getProjMat();        
        customtype::TransformSE3 out_transform = slam_utils::estimateRelativeTransformBtwnProjections(src_proj,tgt_proj);
        status_good = true; // TEMPORARY
        variance = 1; // TEMPORARY


        // customtype::KeyPoints src_kpts = data_spot_src->getKeyPoints();
        // customtype::KeyPoints tgt_kpts = data_spot_target->getKeyPoints();

        customtype::WorldPtsType src_wrldpts;
        customtype::WorldPtsType tgt_wrldpts;
        // customtype::

        findMatchingWorldpoints(data_spot_src->getImageColor(), data_spot_target->getImageColor(), data_spot_src->getKeyPoints(), data_spot_target->getKeyPoints(), data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints(), src_wrldpts, tgt_wrldpts);//, data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints())

        customtype::PointCloudPtr src_cloud(new customtype::PointCloud());
        customtype::PointCloudPtr tgt_cloud(new customtype::PointCloud());

        src_cloud = convert3dPointsToCloud(src_wrldpts);
        tgt_cloud = convert3dPointsToCloud(tgt_wrldpts);
        

        // src_cloud

        // customtype::ImgPtsType 
        // return out_transform;


    }

    customtype::PointCloudPtr TransformEstimator::convert3dPointsToCloud(customtype::WorldPtsType wrldpts)
    {

        customtype::PointCloudPtr cloud(new customtype::PointCloud());
        cloud->height = 1;
        cloud->width  = wrldpts.size();
        cloud->is_dense = false;

        cloud->resize(cloud->height * cloud->width);

        for (int i = 0; i < world_points.size();)
        {

            customtype::CloudPoint & point = cloud->at(i);
            point.x = world_points.at(i).x;
            point.y = world_points.at(i).y;
            point.z = world_points.at(i).z;
            std::cout << i << std::endl;
            ++i;

        }

        return cloud;
    }

    void TransformEstimator::findMatchingWorldpoints(cv::Mat& image1, cv::Mat& image2, 
                                                     customtype::KeyPoints keypoints1,
                                                     customtype::KeyPoints keypoints2, 
                                                     customtype::WorldPtsType wrldpts1,
                                                     customtype::WorldPtsType wrldpts2,
                                                     customtype::WorldPtsType& out_1,
                                                     customtype::WorldPtsType& out_2)
    {

        cv::Mat descriptors1, descriptors2;
        extractor_->compute(image1,keypoints1,descriptors1);
        extractor_->compute(image2,keypoints2,descriptors2);
        // 2. Match the two image descriptors
        // Construction of the matcher

        if(descriptors1.type()!=CV_32F) 
        {
            descriptors1.convertTo(descriptors1, CV_32F);
        }

        if(descriptors2.type()!=CV_32F) 
        {
            descriptors2.convertTo(descriptors2, CV_32F);
        }

         cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce" FlannBased

        // from image 1 to image 2
        // based on k nearest neighbours (with k=2)
        std::vector< std::vector<cv::DMatch> > matches1;
        matcher->knnMatch(descriptors1,descriptors2,
                         matches1, // vector of matches (up to 2 per entry)
                         2);
        // return 2 nearest neighbours
        // from image 2 to image 1
        // based on k nearest neighbours (with k=2)
        std::vector< std::vector<cv::DMatch> > matches2;
        matcher->knnMatch(descriptors2,descriptors1,
                         matches2, // vector of matches (up to 2 per entry)
                         2);
        // return 2 nearest neighbours
        // 3. Remove matches for which NN ratio is
        // > than threshold
        // clean image 1 -> image 2 matches
        int removed= ratioTest(matches1);
        // clean image 2 -> image 1 matches
        removed= ratioTest(matches2);
        // 4. Remove non-symmetrical matches
        std::vector<cv::DMatch> symMatches;
        symmetryTest(matches1,matches2,symMatches);

        std::vector<cv::DMatch> matches;
        // 5. Validate matches using RANSAC
        cv::Mat fundemental = ransacTest(symMatches,
                        keypoints1, keypoints2, matches);



        for (std::vector<cv::DMatch>::
                 const_iterator it= matches.begin();
                 it!= matches.end(); ++it) 
        {
            out_1.push_back(wrldpts1.at(it->queryIdx));
            out_2.push_back(wrldpts2.at(it->trainIdx));
        }
        // return 0;


    }
    


    void TransformEstimator::findMatches(DataSpot3D::DataSpot3DPtr spot_src, DataSpot3D::DataSpot3DPtr spot_target, std::vector<cv::DMatch>& matches)
    {

        findMatches(spot_src->getImageColor(), spot_target->getImageColor(),
              matches, spot_src->getKeyPoints(), spot_target->getKeyPoints());
    }

    // Match feature points using symmetry test and RANSAC
    // returns fundemental matrix
    cv::Mat TransformEstimator::findMatches(cv::Mat& image1,
                       cv::Mat& image2, // input images
                       // output matches and keypoints
                       std::vector<cv::DMatch>& matches,
                       std::vector<cv::KeyPoint>& keypoints1,
                       std::vector<cv::KeyPoint>& keypoints2) 

    {
        // 1a. Detection of the SURF features
        if( !keypoints1.size() ) detector_->detect(image1,keypoints1);
        if( !keypoints2.size() ) detector_->detect(image2,keypoints2);

        if( keypoints1.size() < 8 || keypoints2.size() < 8)
            return cv::Mat();

        // 1b. Extraction of the SURF descriptors
        cv::Mat descriptors1, descriptors2;
        extractor_->compute(image1,keypoints1,descriptors1);
        extractor_->compute(image2,keypoints2,descriptors2);
        // 2. Match the two image descriptors
        // Construction of the matcher

        if(descriptors1.type()!=CV_32F) 
        {
            descriptors1.convertTo(descriptors1, CV_32F);
        }

        if(descriptors2.type()!=CV_32F) 
        {
            descriptors2.convertTo(descriptors2, CV_32F);
        }

         cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce" FlannBased

        // from image 1 to image 2
        // based on k nearest neighbours (with k=2)
        std::vector< std::vector<cv::DMatch> > matches1;
        matcher->knnMatch(descriptors1,descriptors2,
                         matches1, // vector of matches (up to 2 per entry)
                         2);
        // return 2 nearest neighbours
        // from image 2 to image 1
        // based on k nearest neighbours (with k=2)
        std::vector< std::vector<cv::DMatch> > matches2;
        matcher->knnMatch(descriptors2,descriptors1,
                         matches2, // vector of matches (up to 2 per entry)
                         2);
        // return 2 nearest neighbours
        // 3. Remove matches for which NN ratio is
        // > than threshold
        // clean image 1 -> image 2 matches
        int removed= ratioTest(matches1);
        // clean image 2 -> image 1 matches
        removed= ratioTest(matches2);
        // 4. Remove non-symmetrical matches
        std::vector<cv::DMatch> symMatches;
        symmetryTest(matches1,matches2,symMatches);
        // 5. Validate matches using RANSAC
        cv::Mat fundemental = ransacTest(symMatches,
                                        keypoints1, keypoints2, matches);
        // return the found fundemental matrix
        return fundemental;
    }


    // Clear matches for which NN ratio is > than threshold
    // return the number of removed points
    // (corresponding entries being cleared,
    // i.e. size will be 0)
    int TransformEstimator::ratioTest(std::vector<std::vector< cv::DMatch> >& matches) 
    {
        int removed=0;
        // for all matches
        for (std::vector<std::vector< cv::DMatch> >::iterator
             matchIterator= matches.begin();
             matchIterator!= matches.end(); ++matchIterator) {
            // if 2 NN has been identified
            if (matchIterator->size() > 1) {
                // check distance ratio
                if ((*matchIterator)[0].distance/
                        (*matchIterator)[1].distance > ratio_) {
                    matchIterator->clear(); // remove match
                    removed++;
                }
            } else { // does not have 2 neighbours
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        return removed;
    }

    // Insert symmetrical matches in symMatches vector
    void TransformEstimator::symmetryTest(
            const std::vector<std::vector< cv::DMatch> >& matches1,
            const std::vector<std::vector< cv::DMatch> >& matches2,
            std::vector<cv::DMatch>& symMatches) 
    {
        // for all matches image 1 -> image 2
        for (std::vector<std::vector< cv::DMatch> >::
             const_iterator matchIterator1= matches1.begin();
             matchIterator1!= matches1.end(); ++matchIterator1) {
            // ignore deleted matches
            if (matchIterator1->size() < 2)
                continue;
            // for all matches image 2 -> image 1
            for (std::vector<std::vector< cv::DMatch> >::
                 const_iterator matchIterator2= matches2.begin();
                 matchIterator2!= matches2.end();
                 ++matchIterator2) {
                // ignore deleted matches
                if (matchIterator2->size() < 2)
                    continue;
                // Match symmetry test
                if ((*matchIterator1)[0].queryIdx ==
                        (*matchIterator2)[0].trainIdx &&
                        (*matchIterator2)[0].queryIdx ==
                        (*matchIterator1)[0].trainIdx) {
                    // add symmetrical match
                    symMatches.push_back(
                                cv::DMatch((*matchIterator1)[0].queryIdx,
                                           (*matchIterator1)[0].trainIdx,
                                           (*matchIterator1)[0].distance));
                    break; // next match in image 1 -> image 2
                }
            }
        }
    }

    // Identify good matches using RANSAC
    // Return fundemental matrix
    cv::Mat TransformEstimator::ransacTest(
            const std::vector<cv::DMatch>& matches,
            const std::vector<cv::KeyPoint>& keypoints1,
            const std::vector<cv::KeyPoint>& keypoints2,
            std::vector<cv::DMatch>& outMatches) 
    {
        // Convert keypoints into Point2f
        std::vector<cv::Point2f> points1, points2;
        for (std::vector<cv::DMatch>::
             const_iterator it= matches.begin();
             it!= matches.end(); ++it) 
        {
            // Get the position of left keypoints
            float x= keypoints1[it->queryIdx].pt.x;
            float y= keypoints1[it->queryIdx].pt.y;
            points1.push_back(cv::Point2f(x,y));
            // Get the position of right keypoints
            x= keypoints2[it->trainIdx].pt.x;
            y= keypoints2[it->trainIdx].pt.y;
            points2.push_back(cv::Point2f(x,y));
        }



        if( points1.size() != points2.size() || points1.size() < 8 || points2.size() < 8 )
        {
            std::cout << " Point list size is different or < 8, aborting fundamental matrix computation. " << std::endl;

            return cv::Mat();
        }

        std::cout << " Computing F with NPoints: " << std::min(points1.size(), points2.size()) <<  std::endl;
        // Compute F matrix using RANSAC
        std::vector<uchar> inliers(points1.size(),0);
        cv::Mat fundemental= cv::findFundamentalMat(
                    cv::Mat(points1),cv::Mat(points2), // matching points
                    inliers,
                    // match status (inlier or outlier)
                    CV_FM_RANSAC, // RANSAC method
                    distance_,
                    // distance to epipolar line
                    confidence_); // confidence probability
        // extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator
                itIn= inliers.begin();
        std::vector<cv::DMatch>::const_iterator
                itM= matches.begin();
        // for all matches
        for ( ;itIn!= inliers.end(); ++itIn, ++itM) 
        {
            if (*itIn) // it is a valid match
            {
                outMatches.push_back(*itM);
            }
        }
        if (refineF_) 
        {
            // The F matrix will be recomputed with
            // all accepted matches
            // Convert keypoints into Point2f
            // for final F computation
            points1.clear();
            points2.clear();
            for (std::vector<cv::DMatch>::
                 const_iterator it= outMatches.begin();
                 it!= outMatches.end(); ++it) 
            {
                // Get the position of left keypoints

                float x= keypoints1[it->queryIdx].pt.x;
                float y= keypoints1[it->queryIdx].pt.y;
                points1.push_back(cv::Point2f(x,y));
                // Get the position of right keypoints
                x= keypoints2[it->trainIdx].pt.x;
                y= keypoints2[it->trainIdx].pt.y;
                points2.push_back(cv::Point2f(x,y));
            }

            if( points1.size() != points2.size() || points1.size() < 8 || points2.size() < 8 )
            {
                std::cout << " Point list size is different or < 8, abording fundamental matrix Re-computation. " << std::endl;
                return cv::Mat();
            }

            std::cout << " Re-computing F " << std::endl;
            // Compute 8-point F from all accepted matches
            fundemental= cv::findFundamentalMat(
                        cv::Mat(points1),cv::Mat(points2), // matches
                        CV_FM_8POINT); // 8-point method
        }
        return fundemental;
    }

} // namespace gSlam


    
