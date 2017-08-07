#include "gslam/slam_utils.h"


namespace gSlam
{

namespace slam_utils
{
    // gives transformation of image 2 to image 1 (tgt_prj to src_prj). Gives same result as tgt->pose.inverse()*src->pose
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

    // ----- Aligns the camera frame of the ArDrone to the body frame as tracked by the mocap system
    customtype::TransformSE3 getFrameAligner()
    {
        customtype::TransformSE3 ground_truth = getTransformation(395.2285, 598.4764, 960.3633, -0.020576022865, 0.662864768588, -0.0584081485361, 0.746173840335);
        customtype::TransformSE3 cam_frame = getTransformation(335.484, 633.562, 725.732, -0.686232, 0.716337, -0.115044, 0.0520781);
        return cam_frame.inverse()*ground_truth;

        // 1500727324 678121089 3990 0.3952285 0.5984764 0.9603633 0.020576022865 -0.662864768588 0.0584081485361 0.746173840335
        // 0 335.484 633.562 725.732 -0.686232 0.716337 -0.115044 0.0520781

        // 0.7620954 0.751126 1.207522 -0.0201935529872 -0.668273782067 0.0372556846314 0.742707462599
        // 684.025 761.131 952.153 -0.673607 0.730574 -0.0861964 0.0713069

    }

    customtype::PointCloudPtr getCleanCloud(customtype::PointCloudPtr cloud_in){
        customtype::PointCloudPtr cloud_out(new customtype::PointCloud());


        for(int i = 0; i < cloud_in->size(); i++){

            customtype::CloudPoint& p = cloud_in->at(i);

            if( p.x != p.x || p.y != p.y || p.z != p.z )
                continue;

            cloud_out->points.push_back(p);
        }

        return cloud_out;
    }

    void computeVariance(const customtype::PointCloudPtr & cloud_source,
                         const customtype::PointCloudPtr & cloud_target,
                         const customtype::TransformSE3& rel_transform, // rel_transform:how much the sensor moved w.r.t to world
                         double maxCorrespondenceDistance,
                         bool * hasConvergedOut,
                         double * variance,
                         int * correspondencesOut){

        // rel_transform.inverse(): how much the world (points) moved w.r.t to the previous sensor pose

        customtype::PointCloudPtr clean_src(new customtype::PointCloud()), clean_tgt(new customtype::PointCloud());
        clean_src = getCleanCloud(cloud_source);
        clean_tgt = getCleanCloud(cloud_target);


        customtype::PointCloudPtr cloud_source_registered(new customtype::PointCloud()), clean_cloud_source_registered(new customtype::PointCloud());
        // std::cout << "TRANSFORM: \n" << rel_transform.inverse().matrix() << std::endl;
        pcl::transformPointCloud(*clean_src, *cloud_source_registered, rel_transform.inverse());

        clean_cloud_source_registered = getCleanCloud(cloud_source_registered);


        std::cout << "Src Size: " << clean_cloud_source_registered->size() << " Tgt Size: " << clean_tgt->size() << std::endl;

        // compute variance
        if((correspondencesOut || variance) && clean_tgt->size() >=3 && clean_cloud_source_registered->size()>=3)
        {
            pcl::registration::CorrespondenceEstimation<customtype::CloudPoint, customtype::CloudPoint, double>::Ptr est;
            est.reset(new pcl::registration::CorrespondenceEstimation<customtype::CloudPoint, customtype::CloudPoint, double>);
            est->setInputTarget(clean_tgt);
            est->setInputSource(clean_cloud_source_registered);
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
                    }

                    //variance
                    std::sort(distances.begin (), distances.end ());
                    double median_error_sqr = distances[distances.size () >> 1];
                    *variance = (2.1981 * median_error_sqr);
                }
                else
                {
                    *hasConvergedOut = false;
                    *variance = -1.0;
                }
            }

            if(correspondencesOut)
            {
                *correspondencesOut = (int)correspondences.size();
            }
        }
    }

    customtype::PointCloudPtr voxelize(
            const customtype::PointCloudPtr & cloud,
            float voxelSize)
    {
        assert(voxelSize > 0);
        customtype::PointCloudPtr output(new customtype::PointCloud());
        pcl::VoxelGrid<customtype::CloudPoint> filter;
        filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        filter.setInputCloud(cloud);
        filter.filter(*output);
        return output;
    }

    customtype::PointCloudPtr sampling(
            const customtype::PointCloudPtr & cloud, int samples)
    {

        assert(samples > 0);
        customtype::PointCloudPtr output(new  customtype::PointCloud());
        pcl::RandomSample<customtype::CloudPoint> filter;
        filter.setSample(samples);
        filter.setInputCloud(cloud);
        filter.filter(*output);
        return output;
    }

    // If "voxel" > 0, "samples" is ignored
    customtype::PointCloudPtr getICPReadyCloud(
            const customtype::PointCloudPtr cloud_in,
            float voxel,
            int samples,
            const customtype::TransformSE3 & transform)
    {

        customtype::PointCloudPtr cloud_out(new customtype::PointCloud());



        if(cloud_in->size())
        {
            if(voxel>0)
            {
                cloud_out = voxelize(cloud_in, voxel);
            }
            else if(samples>0 && (int)cloud_in->size() > samples)
            {
                cloud_out = sampling(cloud_in, samples);
            }

            if(cloud_out->size())
            {
                pcl::transformPointCloud (*cloud_out, *cloud_out, transform.matrix().cast<float>());
            }
            else if( cloud_in->size() )
            {
                pcl::transformPointCloud (*cloud_in, *cloud_out, transform.matrix().cast<float>());
            }
        }


        return cloud_out;
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
            double * varianceOut,
            bool& got_transform)
    {
        //NOTE: this method is a mix of two methods:
        //  - getRemainingCorrespondences() in pcl/registration/impl/correspondence_rejection_sample_consensus.hpp
        //  - refineModel() in pcl/sample_consensus/sac.h

        // inlierThreshold = 10;
        // iterations = 10000;
        got_transform = false;
        if(varianceOut)
        {
            *varianceOut = 1.0;
        }
        Eigen::Matrix4d transform;
        if(cloud1->size() >= 3 && cloud1->size() == cloud2->size())
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

            std::cout << source_indices.size() << " source indices size"<< std::endl;
            std::cout << target_indices.size() << " target indices size " << std::endl;

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
                std::cout << inliers.size() << "inliers size " << std::endl;
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
                    // std::cout << bestTransformation << std::endl;
                    transform = bestTransformation.cast<double>();
                    std::ostringstream ss;
                    ss << transform;
                    printf("DEBUG: RANSAC inliers =%d/%d tf =\n%s\n", (int)inliers.size(), (int)cloud1->size(),ss.str().c_str());
                    got_transform = true;
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

    std::string getSlamParameterInfo(SlamParameters::SLAMinfo::SLAMinfoPtr info)
    {
        std::string scene_info = "Scene No: " + std::to_string(info->dataset_id_);
        std::string frames = "Frames Processed: " + std::to_string(info->frames_processed_);
        std::string exit_status = (info->process_success_)?" STAM Success":" STAM failed due to correspondence loss";

        std::string baseline = "VO Triangulation Baseline: " + std::to_string(info->visual_odometry_baseline_);

        std::string optimisation = (info->optimisation_thread_on_==true)?"ON":"OFF";

        std::string loop_closure_info = "Loop Closure Constraint Info = (1*"+std::to_string(info->loopclosure_constraint.const1_)+")/(variance*"+std::to_string(info->loopclosure_constraint.const2_)+")";
        std::string odometry_constraint_info = "Odometry Constraint Info = (1*"+std::to_string(info->odometry_constraint.const1_)+")/(variance*"+std::to_string(info->odometry_constraint.const2_)+")";

        std::string matcher = "Repeated Matches Required: min " + std::to_string(info->matcher_min_repetition_)+ ", max " + std::to_string(info->matcher_max_repetition_);

        std::string icp = "Loop Closure Transformation ICP Parameters:-";
        if (info->transform_est_icp.parameters_defined_)
            icp += "\n  --Inlier Threshold: "+std::to_string(info->transform_est_icp.inlier_threshold_)+"; Max Iterations: " + std::to_string(info->transform_est_icp.max_iterations_)+ "; Refinement Sigma: " + std::to_string(info->transform_est_icp.refine_sigma_)+ "; Max Refinement Iterations: " + std::to_string(info->transform_est_icp.refine_max_iterations_);
        else icp += " ICP was not used";

        std::string optical_flow = "LK Optical Flow Parameters:-";
        if (info->lk_parameters.parameters_defined_)
            optical_flow += " Error Tolerence: " + std::to_string(info->lk_parameters.max_correspondence_error_)+"; Min Matches Required: " + std::to_string(info->lk_parameters.min_correspondences_required_) +"; Max Features Used: " + std::to_string(info->lk_parameters.max_correspondence_used_);
        else optical_flow += " Optical Flow was not used";

        std::string fabmap = "FabMap Modifiers:- First Image Frame: " + std::to_string(info->fabmap.first_bow_img_) + "; Frames Skipped: " + std::to_string(info->fabmap.skip_);

        // std::cout << optimisation << std::endl;
        std::string ret_val = "-"+scene_info + "; " + "; " + frames + "; " + exit_status + "\n" + "-"+ baseline + "; Graph Optimization: " +  optimisation + "\n" + "-"+ loop_closure_info + "\n" + "-"+ odometry_constraint_info + "\n" + "-"+ fabmap + "\n" + "-"+ matcher + "\n" + "-"+ icp + "\n" + "-"+ optical_flow;

        return ret_val;
    }

    cv::Mat calcProjMatrix(std::vector<cv::Point2f> points2d, customtype::WorldPtsType points3d, cv::Mat intrinsics, cv::Mat distortion) 
    {

        // ESTIMATING PROJECTION MATRIX USING PNP-RANSAC. ADAPTED AND MODIFIED FROM STAM.

        assert(points2d.size() == points3d.size());

        cv::Mat guess_r, guess_t;
        customtype::WorldPtsType points3d_new = points3d;
        std::vector<cv::Point2f> points2d_new = points2d;

        // std::cout << "end " << points2d.end();
        int max_size = 100;
        if (points2d.size() > 100)
        {
            std::vector<cv::Point2f>::const_iterator first = points2d.end() - max_size;
            std::vector<cv::Point2f>::const_iterator last = points2d.end();
            std::vector<cv::Point2f> new_2d(first, last);
            points2d_new = new_2d;

            std::vector<cv::Point3f>::const_iterator first3 = points3d.end() - max_size;
            std::vector<cv::Point3f>::const_iterator last3 = points3d.end();
            std::vector<cv::Point3f> new_3d(first3, last3);
            points3d_new = new_3d;
        }


        // ---------- Estimate Projection Matrix (rotation and translation vectors) by solving the PnP problem with ransac
        cv::solvePnPRansac(points3d_new, points2d_new, intrinsics, distortion, guess_r, guess_t, false, 100, 5.0, 0.99);
        cv::Mat rvec = guess_r, tvec = guess_t;
        cv::Mat R(3, 3, CV_64FC1);
        cv::Rodrigues(rvec, R);

        cv::Mat Pose(3,4, R.type());
        R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
        cv::Mat projMatrix = Pose;

        // std::cout << R << std::endl << tvec << std::endl << Pose << std::endl;


    //    printProjMatrix();

        return projMatrix;
    }

    customtype::TransformSE3 estimatePoseFromProjection(cv::Mat projmat)
    {
        cv::Mat R1 = projmat(cv::Range::all(), cv::Range(0, 3));
        cv::Mat T1 = projmat(cv::Range::all(), cv::Range(3, 4));

        // ----- Estimating Camera Pose (Pose from projection)
        cv::Mat Pose(3, 4, CV_64FC1);
        cv::Mat pos, R;
        R = R1.inv();
        pos = (-R) * T1;

        // ----- Creating pose matrix 
        R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
        pos.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
        cv::Mat row = (cv::Mat_<double>(1,4) << 0, 0, 0, 1);
        Pose.push_back(row);

        // std::cout << Pose << std::endl;

        // ----- Converting pose from cv::Mat to Eigen::Transform()
        customtype::TransformSE3 eigen_pose;
        cv::cv2eigen(Pose, eigen_pose.matrix());

        // std::cout << eigen_pose.matrix() << std::endl;

        return eigen_pose;

    }

    


} // namespace slam_utils
} // namespace gSlam