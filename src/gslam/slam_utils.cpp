#include "gslam/slam_utils.h"


namespace gSlam
{

namespace slam_utils
{
    // gives transformation of image 2 to image 1 (tgt_prj to src_prj)
    customtype::TransformSE3 estimateRelativeTransformBtwnProjections(customtype::ProjMatType src_prj, customtype::ProjMatType tgt_prj)
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

    void getTranslationAndEulerAngles (const customtype::TransformSE3& t,
                                   float& x, float& y, float& z,
                                   float& roll, float& pitch, float& yaw)
    {
        x = t(0,3);
        y = t(1,3);
        z = t(2,3);
        roll  = atan2f(t(2,1), t(2,2));
        pitch = asinf(-t(2,0));
        yaw   = atan2f(t(1,0), t(0,0));
    }

    customtype::TransformSE3 getTransformation (double x, double y, double z, double roll, double pitch, double yaw)
    {
        customtype::TransformSE3 t;
        getTransformation (x, y, z, roll, pitch, yaw, t);
        return (t);
    }


    customtype::TransformSE3 getTransformation (double x, double y, double z, double qx, double qy, double qz, double qw) {

        Eigen::Quaternion<double> q;
        q.x() = qx; q.y() = qy; q.z() = qz ; q.w() = qw;
        Eigen::Vector3d t(x, y, z);
        Eigen::Translation<double, 3> trans(t);
        customtype::TransformSE3 transform = trans*q;

        return transform;

    }

    // Get transform from cloud2 to cloud1
    Eigen::Matrix4d transformFromXYZCorrespondences(
            const customtype::PointCloudPtr & cloud1,
            const customtype::PointCloudPtr & cloud2,
            double inlierThreshold,
            int iterations,
            bool refineModel,
            double refineModelSigma,
            int refineModelIterations,
            std::vector<int> * inliersOut,
            double * varianceOut)
    {
        //NOTE: this method is a mix of two methods:
        //  - getRemainingCorrespondences() in pcl/registration/impl/correspondence_rejection_sample_consensus.hpp
        //  - refineModel() in pcl/sample_consensus/sac.h

        if(varianceOut)
        {
            *varianceOut = 1.0;
        }
        Eigen::Matrix4d transform;
        if(cloud1->size() >=3 && cloud1->size() == cloud2->size())
        {
            // RANSAC
            printf("DEBUG: iterations=%d inlierThreshold=%f\n", iterations, inlierThreshold);
            std::vector<int> source_indices (cloud2->size());
            std::vector<int> target_indices (cloud1->size());

            // Copy the query-match indices
            for (int i = 0; i < (int)cloud1->size(); ++i)
            {
                source_indices[i] = i;
                target_indices[i] = i;
            }

            // From the set of correspondences found, attempt to remove outliers
            // Create the registration model
            pcl::SampleConsensusModelRegistration<customtype::CloudPoint>::Ptr model;
            model.reset(new pcl::SampleConsensusModelRegistration<customtype::CloudPoint>(cloud2, source_indices));
            // Pass the target_indices
            model->setInputTarget (cloud1, target_indices);
            // Create a RANSAC model
            pcl::RandomSampleConsensus<customtype::CloudPoint> sac (model, inlierThreshold);
            sac.setMaxIterations(iterations);

            // Compute the set of inliers
            if(sac.computeModel())
            {
                std::vector<int> inliers;
                Eigen::VectorXf model_coefficients;

                sac.getInliers(inliers);
                sac.getModelCoefficients (model_coefficients);

                if (refineModel)
                {
                    double inlier_distance_threshold_sqr = inlierThreshold * inlierThreshold;
                    double error_threshold = inlierThreshold;
                    double sigma_sqr = refineModelSigma * refineModelSigma;
                    int refine_iterations = 0;
                    bool inlier_changed = false, oscillating = false;
                    std::vector<int> new_inliers, prev_inliers = inliers;
                    std::vector<size_t> inliers_sizes;
                    Eigen::VectorXf new_model_coefficients = model_coefficients;
                    do
                    {
                        // Optimize the model coefficients
                        model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
                        inliers_sizes.push_back (prev_inliers.size ());

                        // Select the new inliers based on the optimized coefficients and new threshold
                        model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);
                        printf("DEBUG: RANSAC refineModel: Number of inliers found (before/after): %d/%d, with an error threshold of %f.\n",
                               (int)prev_inliers.size (), (int)new_inliers.size (), error_threshold);

                        if (new_inliers.empty ())
                        {
                            ++refine_iterations;
                            if (refine_iterations >= refineModelIterations)
                            {
                                break;
                            }
                            continue;
                        }

                        // Estimate the variance and the new threshold
                        double variance = model->computeVariance ();
                        error_threshold = sqrt (std::min (inlier_distance_threshold_sqr, sigma_sqr * variance));

                        printf ("DEBUG: RANSAC refineModel: New estimated error threshold: %f (variance=%f) on iteration %d out of %d.\n",
                                error_threshold, variance, refine_iterations, refineModelIterations);
                        inlier_changed = false;
                        std::swap (prev_inliers, new_inliers);

                        // If the number of inliers changed, then we are still optimizing
                        if (new_inliers.size () != prev_inliers.size ())
                        {
                            // Check if the number of inliers is oscillating in between two values
                            if (inliers_sizes.size () >= 4)
                            {
                                if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                                        inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
                                {
                                    oscillating = true;
                                    break;
                                }
                            }
                            inlier_changed = true;
                            continue;
                        }

                        // Check the values of the inlier set
                        for (size_t i = 0; i < prev_inliers.size (); ++i)
                        {
                            // If the value of the inliers changed, then we are still optimizing
                            if (prev_inliers[i] != new_inliers[i])
                            {
                                inlier_changed = true;
                                break;
                            }
                        }
                    }
                    while (inlier_changed && ++refine_iterations < refineModelIterations);

                    // If the new set of inliers is empty, we didn't do a good job refining
                    if (new_inliers.empty ())
                    {
                        printf ("WARN: RANSAC refineModel: Refinement failed: got an empty set of inliers!\n");
                    }

                    if (oscillating)
                    {
                        printf("DEBUG: RANSAC refineModel: Detected oscillations in the model refinement.\n");
                    }

                    std::swap (inliers, new_inliers);
                    model_coefficients = new_model_coefficients;
                }

                if (inliers.size() >= 3)
                {
                    if(inliersOut)
                    {
                        *inliersOut = inliers;
                    }
                    if(varianceOut)
                    {
                        *varianceOut = model->computeVariance();
                    }

                    // get best transformation
                    Eigen::Matrix4f bestTransformation;
                    bestTransformation.row (0) = model_coefficients.segment<4>(0);
                    bestTransformation.row (1) = model_coefficients.segment<4>(4);
                    bestTransformation.row (2) = model_coefficients.segment<4>(8);
                    bestTransformation.row (3) = model_coefficients.segment<4>(12);

                    transform = bestTransformation.cast<double>();
                    std::ostringstream ss;
                    ss << transform;
                    printf("DEBUG: RANSAC inliers=%d/%d tf=%s", (int)inliers.size(), (int)cloud1->size(),ss.str().c_str());

                    return transform.inverse(); // inverse to get actual pose transform (not correspondences transform)
                }
                else
                {
                    printf("DEBUG: RANSAC: Model with inliers < 3\n");
                }
            }
            else
            {
                printf("DEBUG: RANSAC: Failed to find model\n");
            }
        }
        else
        {
            printf("DEBUG: Not enough points to compute the transform\n");
        }
        return Eigen::Matrix4d();
    }

    // // return transform from source to target (All points must be finite!!!)
    // Eigen::Matrix4d icp2D(const customtype::PointCloudPtr & cloud_source,
    //                       const customtype::PointCloudPtr & cloud_target, //TODO: Should be ConstPtr
    //                       double maxCorrespondenceDistance,
    //                       int maximumIterations,
    //                       bool * hasConvergedOut,
    //                       double * variance,
    //                       int * correspondencesOut)
    // {
    //     pcl::IterativeClosestPoint<customtype::CloudPoint, customtype::CloudPoint, double> icp;
    //     // Set the input source and target
    //     icp.setInputTarget (cloud_target);
    //     icp.setInputSource (cloud_source);

    //     pcl::registration::TransformationEstimation2D<customtype::CloudPoint, customtype::CloudPoint, double>::Ptr est;
    //     est.reset(new pcl::registration::TransformationEstimation2D<customtype::CloudPoint, customtype::CloudPoint, double>);
    //     icp.setTransformationEstimation(est);

    //     // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    //     icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
    //     // Set the maximum number of iterations (criterion 1)
    //     icp.setMaximumIterations (maximumIterations);
    //     // Set the transformation epsilon (criterion 2)
    //     //icp.setTransformationEpsilon (transformationEpsilon);
    //     // Set the euclidean distance difference epsilon (criterion 3)
    //     //icp.setEuclideanFitnessEpsilon (1);
    //     //icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

    //     // Perform the alignment
    //     customtype::PointCloudPtr cloud_source_registered(new customtype::PointCloud());
    //     icp.align (*cloud_source_registered);
    //     bool hasConverged = icp.hasConverged();

    //     // compute variance
    //     if((correspondencesOut || variance) && hasConverged)
    //     {
    //         pcl::registration::CorrespondenceEstimation<customtype::CloudPoint, customtype::CloudPoint, double>::Ptr est;
    //         est.reset(new pcl::registration::CorrespondenceEstimation<customtype::CloudPoint, customtype::CloudPoint, double>);
    //         est->setInputTarget(cloud_target);
    //         est->setInputSource(cloud_source_registered);
    //         pcl::Correspondences correspondences;
    //         est->determineCorrespondences(correspondences, maxCorrespondenceDistance);
    //         if(variance)
    //         {
    //             if(correspondences.size()>=3)
    //             {
    //                 std::vector<double> distances(correspondences.size());
    //                 for(unsigned int i=0; i<correspondences.size(); ++i)
    //                 {
    //                     distances[i] = correspondences[i].distance;
    //                 }

    //                 //variance
    //                 std::sort(distances.begin (), distances.end ());
    //                 double median_error_sqr = distances[distances.size () >> 1];
    //                 *variance = (2.1981 * median_error_sqr);
    //             }
    //             else
    //             {
    //                 hasConverged = false;
    //                 *variance = -1.0;
    //             }
    //         }

    //         if(correspondencesOut)
    //         {
    //             *correspondencesOut = (int)correspondences.size();
    //         }
    //     }
    //     else
    //     {
    //         if(correspondencesOut)
    //         {
    //             *correspondencesOut = 0;
    //         }
    //         if(variance)
    //         {
    //             *variance = -1;
    //         }
    //     }

    //     if(hasConvergedOut)
    //     {
    //         *hasConvergedOut = hasConverged;
    //     }

    //     Eigen::Matrix4d m = icp.getFinalTransformation();
    //     //slam_x::TransformSE3 rel_transform = icp.getFinalTransformation().matrix();
    //     return m;
    // }

    customtype::PointCloudPtr convert3dPointsToCloud(customtype::WorldPtsType wrldpts)
    {

        customtype::PointCloudPtr cloud(new customtype::PointCloud());
        cloud->height = 1;
        cloud->width  = wrldpts.size();
        cloud->is_dense = false;
        // for(int i = 0 ; i < wrldpts.size(); ++i)
        // std::cout << " wrldpts " << wrldpts.at(i).x << " " << wrldpts.at(i).y <<" " << wrldpts.at(i).z << std::endl;

        cloud->resize(cloud->height * cloud->width);

        for (int i = 0; i < wrldpts.size();)
        {

            customtype::CloudPoint & point = cloud->at(i);
            point.x = wrldpts.at(i).x;
            point.y = wrldpts.at(i).y;
            point.z = wrldpts.at(i).z;
            // std::cout << i << std::endl;
            ++i;

        }

        return cloud;
    }

    // return transform from source to target (All points must be finite!!!)
    Eigen::Matrix4d icp(const customtype::PointCloudPtr & cloud_source, //TODO: Should be ConstPtr
                        const customtype::PointCloudPtr & cloud_target,
                        double maxCorrespondenceDistance,
                        int maximumIterations,
                        bool * hasConvergedOut,
                        double * variance,
                        int * correspondencesOut)
    {
        pcl::IterativeClosestPoint<customtype::CloudPoint, customtype::CloudPoint, double> icp;
        // Set the input source and target
        icp.setInputTarget (cloud_target);
        icp.setInputSource (cloud_source);

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (maximumIterations);
        // Set the transformation epsilon (criterion 2)
        //icp.setTransformationEpsilon (transformationEpsilon);
        // Set the euclidean distance difference epsilon (criterion 3)
        //icp.setEuclideanFitnessEpsilon (1);
        //icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

        // Perform the alignment
        customtype::PointCloudPtr cloud_source_registered(new customtype::PointCloud());
        icp.align (*cloud_source_registered);
        bool hasConverged = icp.hasConverged();
        *variance = 1;

        // compute variance
        if((correspondencesOut || variance) && hasConverged)
        {
            pcl::registration::CorrespondenceEstimation<customtype::CloudPoint, customtype::CloudPoint, double>::Ptr est;
            est.reset(new pcl::registration::CorrespondenceEstimation<customtype::CloudPoint, customtype::CloudPoint, double>);
            est->setInputTarget(cloud_target);
            est->setInputSource(cloud_source_registered);
            pcl::Correspondences correspondences;
            est->determineCorrespondences(correspondences, maxCorrespondenceDistance);
            if(variance)
            {
                if(correspondences.size()>=3)
                {
                    std::vector<double> distances(correspondences.size());
                    for(unsigned int i=0; i<correspondences.size(); ++i)
                    {
                        distances[i] = correspondences[i].distance;
                        // std::cout << " distance " << correspondences[i].distance << std::endl;
                    }

                    //variance
                    std::sort(distances.begin (), distances.end ());
                    // std::cout << correspondences.size() << std::endl;
                    double median_error_sqr = distances[distances.size () >> 1];
                    *variance = (2.1981 * median_error_sqr);
                    // std::cout << "REACHING HERE!!! " << median_error_sqr << " " << *variance << std::endl;
                }
                else
                {
                    hasConverged = false;
                    *variance = -1.0;
                }
            }

            if(correspondencesOut)
            {
                *correspondencesOut = (int)correspondences.size();
            }
        }
        else
        {
            if(correspondencesOut)
            {
                *correspondencesOut = 0;
            }
            if(variance)
            {
                *variance = -1;
            }
        }

        if(hasConvergedOut)
        {
            *hasConvergedOut = hasConverged;
        }

        Eigen::Matrix4d m = icp.getFinalTransformation();
        //slam_x::TransformSE3 rel_transform = icp.getFinalTransformation().matrix();
        return m;
    }




    // CLASS DATASPOTMATCHER -----------

    void DataSpotMatcher::findMatches(DataSpot3D::DataSpot3DPtr spot_src, DataSpot3D::DataSpot3DPtr spot_target, std::vector<cv::DMatch>& matches)
    {

        findMatches(spot_src->getImageColor(), spot_target->getImageColor(),
              matches, spot_src->getKeyPoints(), spot_target->getKeyPoints());
    }

    // Match feature points using symmetry test and RANSAC
    // returns fundemental matrix
    cv::Mat DataSpotMatcher::findMatches(cv::Mat& image1,
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
    int DataSpotMatcher::ratioTest(std::vector<std::vector< cv::DMatch> >& matches) 
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
    void DataSpotMatcher::symmetryTest(
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
    cv::Mat DataSpotMatcher::ransacTest(
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

    void DataSpotMatcher::findMatchingWorldpoints(cv::Mat image1, cv::Mat image2, 
                                                 customtype::KeyPoints imgpts1,
                                                 customtype::KeyPoints imgpts2, 
                                                 customtype::WorldPtsType wrldpts1,
                                                 customtype::WorldPtsType wrldpts2,
                                                 customtype::WorldPtsType& out_1,
                                                 customtype::WorldPtsType& out_2)
    {

        cv::cvtColor(image1, image1, CV_BGR2GRAY);
        cv::cvtColor(image2, image2, CV_BGR2GRAY);

        // detector_->detect(image1,imgpts1);
        // detector_->detect(image2,imgpts2);

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
        if (((final_matches.size()>(0.19*float(imgpts1.size())) && final_matches.size()>(0.19*float(imgpts2.size()))) && (imgpts1.size()>200 && imgpts2.size()>200))||final_matches.size() > 100)
        {
            // cv::Mat out_img;
            // cv::drawKeypoints(image1, imgpts1, out_img);
            // cv::imshow("window", out_img);
            // cv::waitKey(1);
            // cv::Mat out_img2;
            // cv::drawKeypoints(image2, imgpts2, out_img2);
            // cv::imshow("window2", out_img2);
            // cv::waitKey(0);

            cv::drawMatches(image1,imgpts1, image2, imgpts2, final_matches, out_match);
            cv::imshow("loop closure matches",out_match);
            cv::waitKey(1);
        }
        // std::cout << "this here" << std::endl;
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
        // return 0;


    }

} // namespace slam_utils
} // namespace gSlam