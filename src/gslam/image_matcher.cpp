/** @file image_matcher.cpp (Class for performing image matching. NOT IN USE CURRENTLY. CAN BE REMOVED WHEN REMOVING PCL)
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

// TODO: REMOVE THIS FILE WHEN REMOVING PCL (NOT REQUIRED)

#include "gslam/image_matcher.h"

namespace gSlam
{
namespace slam_utils
{

    // CLASS ImageMatcher -----------

    void ImageMatcher::findMatches(DataSpot3D::DataSpot3DPtr spot_src, DataSpot3D::DataSpot3DPtr spot_target, std::vector<cv::DMatch>& matches)
    {

        findMatches(spot_src->getImageColor(), spot_target->getImageColor(),
              matches, spot_src->getKeyPoints(), spot_target->getKeyPoints());
    }

    // Match feature points using symmetry test and RANSAC
    // returns fundemental matrix
    cv::Mat ImageMatcher::findMatches(cv::Mat& image1,
                       cv::Mat& image2, // input images
                       // output matches and keypoints
                       std::vector<cv::DMatch>& matches,
                       std::vector<cv::KeyPoint>& keypoints1,
                       std::vector<cv::KeyPoint>& keypoints2) 

    {
        // 1a. Detection of the SURF features
        // std::cout << "reaching here!!!" << std::endl;
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
    int ImageMatcher::ratioTest(std::vector<std::vector< cv::DMatch> >& matches) 
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
    void ImageMatcher::symmetryTest(
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
    cv::Mat ImageMatcher::ransacTest(
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
            fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), // matches
                        CV_FM_8POINT); // 8-point method
            // std::cout << points2.size() << ' ' << points1.size() << std::endl;
        }
        return fundemental;
    }

    // not used because images are already undistorted
    void ImageMatcher::findMatchingWorldpoints(DataSpot3D::DataSpot3DPtr data_spot_src, DataSpot3D::DataSpot3DPtr data_spot_target,
                                                  customtype::WorldPtsType& src_wrldpts, customtype::WorldPtsType& tgt_wrldpts, bool& good_match_status)
    {
        cv::Mat src_img, tgt_img;
        // std::cout << data_spot_src->getCamParams().intrinsics_ << std::endl;
        // std::cout << data_spot_src->getCamParams().distortion_ << std::endl;
        
        cv::undistort(data_spot_src->getImageColor(), src_img, data_spot_src->getCamParams().intrinsics_, data_spot_src->getCamParams().distortion_);
        cv::undistort(data_spot_target->getImageColor(), tgt_img, data_spot_target->getCamParams().intrinsics_, data_spot_target->getCamParams().distortion_);

        src_img = data_spot_src->getImageColor(); tgt_img = data_spot_target->getImageColor();
        findMatchingWorldpoints(src_img, tgt_img, data_spot_src->getImagePoints(), data_spot_target->getImagePoints(), data_spot_src->getWorldPoints(), data_spot_target->getWorldPoints(), src_wrldpts, tgt_wrldpts, good_match_status);

        // cv::imshow("window 1", src_img);
        // cv::waitKey(1);
        // cv::imshow("window 2", data_spot_src->getImageColor());
        // cv::waitKey(0);

    }

    void ImageMatcher::findMatchingWorldpoints(cv::Mat image1, cv::Mat image2, 
                                                 customtype::KeyPoints imgpts1,
                                                 customtype::KeyPoints imgpts2, 
                                                 customtype::WorldPtsType wrldpts1,
                                                 customtype::WorldPtsType wrldpts2,
                                                 customtype::WorldPtsType& out_1,
                                                 customtype::WorldPtsType& out_2,
                                                 bool& good_match)
    {

        cv::cvtColor(image1, image1, CV_BGR2GRAY);
        cv::cvtColor(image2, image2, CV_BGR2GRAY);

        // cv::undistort()
        // cv2.countNonZero(cv2.subtract(self.mat,cv_img))>0
        // if (!prev_frame_.empty())
        // {
        //     cv::Mat diff_val;
        //     cv::subtract(prev_frame_, image1, diff_val);
        //     if (cv::countNonZero(diff_val)==0)
        //     {
        //         repeat_match_count_ ++;
        //         std::cout << "--------------------------------------------------repeating match x"<<repeat_match_count_ << std::endl;
        //     }
        //     else repeat_match_count_ = 0;
        // }
        // detector_->detect(image1,imgpts1);
        // detector_->detect(image2,imgpts2);

        good_match = false;

        // if (repeat_match_count > 0)
        //     std::cout << "--------------------------------------------------repeating match x"<<repeat_match_count << std::endl;

        cv::Mat descriptors1, descriptors2;
        extractor_->compute(image1,imgpts1,descriptors1);
        extractor_->compute(image2,imgpts2,descriptors2);
        // std::cout << " size " << imgpts2.size() << std::endl;
        // std::cout << " size " << imgpts1.size()<< std::endl;
        // std::cout  << "size " << wrldpts1.size() << std::endl;
        // std::cout << " size " << wrldpts2.size() << std::endl;
        // 2. Match the two image descriptors
        // Construction of the matcher

        // if(descriptors1.type()!=CV_32F) 
        // {
        //     descriptors1.convertTo(descriptors1, CV_32F);
        // }

        // if(descriptors2.type()!=CV_32F) 
        // {
        //     descriptors2.convertTo(descriptors2, CV_32F);
        // }

         cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce" FlannBased

        // from image 1 to image 2
        // based on k nearest neighbours (with k=2)
        std::vector< std::vector<cv::DMatch> > matches1;
        matcher->knnMatch(descriptors1,descriptors2,
                         matches1, // vector of matches (up to 2 per entry)
                         5);
        // return 2 nearest neighbours
        // from image 2 to image 1
        // based on k nearest neighbours (with k=2)
        std::vector< std::vector<cv::DMatch> > matches2;
        matcher->knnMatch(descriptors2,descriptors1,
                         matches2, // vector of matches (up to 2 per entry)
                         5);
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
        // std::cout << symMatches.size() << "symsize" << std::endl;

        std::vector<cv::DMatch> good_matches, final_matches;
        // 5. Validate matches using RANSAC // (if using, change symMatches to matches in the for loop below)
        // cv::Mat fundemental = ransacTest(symMatches,
        //                 imgpts1, imgpts2, good_matches);

        // for (int i = 0; i<imgpts2.size(); ++i)
        //     {
        //     }
        // std::cout << "imgpts1 size " << imgpts1.size() << std::endl;
        // std::cout << "imgpts2 size " << imgpts2.size() << std::endl;
        final_matches = symMatches;
        // std::cout << symMatches.size() << " matches out of "<<imgpts1.size() << " and " << imgpts2.size() << " (" << std::endl;
        printf("%lu matches out of %lu and %lu (%f%% , %f%%)\n",final_matches.size(), imgpts1.size(), imgpts2.size(), (float(final_matches.size())/float(imgpts1.size()))*100.0,(float(final_matches.size())/float(imgpts2.size()))*100.0);
        // printf("%s\n", );

        cv::Mat out_match;
        if (
            ((final_matches.size()>(0.18*float(imgpts1.size())) && final_matches.size()>(0.18*float(imgpts2.size()))) && (imgpts1.size()>200 && imgpts2.size()>200))
            ||final_matches.size() > 80 
            // || (repeat_match_count >= min_repeat_match_count_)// && final_matches.size()>50)
            // || (final_matches.size()>50 && (final_matches.size()>(0.2*float(imgpts1.size()))||(final_matches.size()>(0.2*float(imgpts1.size())) ) ))
           )
        {
            // cv::Mat out_img;
            // cv::drawKeypoints(image1, imgpts1, out_img);
            // cv::imshow("window", out_img);
            // cv::waitKey(1);
            // cv::Mat out_img2;
            // cv::drawKeypoints(image2, imgpts2, out_img2);
            // cv::imshow("window2", out_img2);
            // cv::drawMatches
            // cv::waitKey(0);
            good_match = true;
            cv::drawMatches(image1,imgpts1, image2, imgpts2, final_matches, out_match);
            cv::imshow("loop closure matches",out_match);
            // cv::waitKey(0);
            for (std::vector<cv::DMatch>::
                     const_iterator it= final_matches.begin();
                     it!= final_matches.end(); ++it) 
            {
                // std::cout << it->queryIdx << " " << it->trainIdx << std::endl;
                out_1.push_back(wrldpts1[it->queryIdx]);
                // std::cout << wrldpts1[it->queryIdx].x << " "  << wrldpts1[it->queryIdx].y << " " << wrldpts1[it->queryIdx].z << " " <<std::endl;
                    // std::cout << "imgpts1 " << imgpts1.at(it->queryIdx).pt.x << " " << imgpts1.at(it->queryIdx).pt.y << std::endl;
                out_2.push_back(wrldpts2[it->trainIdx]);
                // std::cout << wrldpts2[it->trainIdx].x << " "  << wrldpts2[it->trainIdx].y << " " << wrldpts2[it->trainIdx].z << " " <<std::endl;
                    // std::cout << "imgpts2 " << imgpts2.at(it->trainIdx).pt.x << " " << imgpts2.at(it->trainIdx).pt.y << std::endl;
            }
        }
        // std::cout << "this here" << std::endl;
        // return 0;


    }

} // namespace slam_utils
} // namespace gSlam