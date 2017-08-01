
#include "gslam/data_pool.h"
#include "gslam/fabmap.h"

namespace gSlam
{


DataPool::DataPool() : loop_count_far_(0), loop_count_near_(0), repeat_match_ (false), prev_loop_id_(-2), odom_drift_(0.01), drift_rate_(0.01){}

void DataPool::addDataSpot(DataSpot3D::DataSpot3DPtr data_spot_ptr)

{
    // std::cout << " Adding data spot " << std::endl;

    // std::cout << " checking loop closure " << std::endl;

    //Loop closure constraints
    // if (data_spot_ptr->getId() == 201)
    // {
    //     cv::Mat out_img;
    //     cv::drawKeypoints(data_spot_ptr->getImageColor(), data_spot_ptr->getImagePoints(), out_img);
    //     cv::imshow("gslam_window", out_img);
    //     cv::waitKey(0);
    // }
    // std::cout << data_spot_ptr->getCamParams().intrinsics_ << std::endl;
    require_optimization_flag_ = false;
    int new_id, loop_id;
    fabmap_.compareAndAdd(data_spot_ptr, new_id, loop_id);
    // std::cout << data_spot_ptr->getImagePoints().size() << "check " << std::endl;
    // std::cout << data_spot_ptr->getWorldPoints().size() << " check" << std::endl;
    assert(data_spot_ptr->getImagePoints().size() == data_spot_ptr->getWorldPoints().size());

    std::cout << "                                                                 LoopID: " << loop_id << " new_id: " << new_id << " prev_loop_id_: " << prev_loop_id_<< std::endl;


    // checking if fabmap detects same frame as loop closure continuously. If detected more times than some threshold, it is probably a true loop closure.
    if (prev_loop_id_ == loop_id)
    {
        repeat_match_ = true;
        // std::cout << "prev_loop_id_ = loop_id ************"<<repeat_match_<<"************************\n";
    }
    else repeat_match_ = false;


    prev_loop_id_ = loop_id; 
    if( loop_id >= 0 && new_id > 0 && (new_id - loop_id) > 50) 
    {
        std::cout << "Possible loop closure : " << new_id << "->" << loop_id << std::endl;
        double variance, prop_matches;
        int correspondences;
        bool status_good = false;
        DataLink3D::DataLinkPtr link( new DataLink3D() );
        link->inf_matrix_ = customtype::InformationMatrix3D::Identity();

        DataSpot3D::DataSpot3DPtr spot_src = data_spots_.find(loop_id)->second;

        link->from_id_ = spot_src->getId();
        link->to_id_ = data_spot_ptr->getId();
        
        std::cout << " Estimating Loop Closure Transform " <<std::endl;
        link->transform_ = transform_est_.estimateTransform(spot_src, data_spot_ptr, variance, correspondences, prop_matches, status_good, repeat_match_);

        // link->transform_ = data_spot_ptr->getPose().inverse()*spot_src->getPose();
        // --- Enforce 2D ---
        // float x,y,z,r,p,yaw;
        // slam_x_slam_utils::getTranslationAndEulerAngles(link->transform_, x, y, z, r, p , yaw);
        // link->transform_ = slam_utils::getTransformation(x, y, 0, 0, 0, yaw);

        if (variance == 0)
            variance = 1.0;
        double info = 1.00/(variance); // before 1.0/(variance*100);

        // info before was 100
        // if( info > 0 && info < 1000000) link->inf_matrix_ *= info*1;
        // else link->inf_matrix_ *= 1;//0.5*(1+prop_matches)*2;

        link->inf_matrix_ *= info; // =========================

        // std::cout << "info matrix loop closure \n" << link->inf_matrix_ << std::endl;
        link->active = true;
        link->type = DataLink3D::LoopClosureConstraint;
        std::cout << "Loop Closure Status: " << std::boolalpha << status_good << std::noboolalpha << ";  LOOP: corr " <<  correspondences << "  variance:  " << variance << "  infor : " << info << std::endl;
        if( status_good )
        {

            // std::cout << link->transform_.matrix() << std::endl;
            // std::cin.get();
            bool valid = true;
            if( (new_id - loop_id) > 400 ) // FAR LOOP
            {
                loop_count_far_++;
                new_count_loop_far_++;
            }
            else 
            { 
                loop_count_near_++;
                new_count_loop_near_++;
            }

            Eigen::Vector3d t0 = (spot_src->getPose()*link->transform_).translation();
            Eigen::Vector3d t1 = data_spot_ptr->getPose().translation();
            float dist = (t1-t0).norm();
            std::cout <<"               DISTANCE:    " <<dist << std::endl;

            spot_src->addLink(link);

            // if (dist > 500)
            require_optimization_flag_ = true; // TODO: add some condition to check if optimization is required.

            std::cout << " LOOP ADDED ! NFar " << loop_count_far_ << " NNear " << loop_count_near_ << std::endl;
            // cv::waitKey(1);

            // std::cout << "Press Return to continue\n " << std::endl;
            // std::cin.get();
        }
    }  // loop closure -- if()

    // std::cout << " Adding odometry " << std::endl;
    // Odometry constraint
    if( last_spot_.get() )
    {
        customtype::TransformSE3 rel_transform = last_spot_->getPose().inverse()*data_spot_ptr->getPose();
        // customtype::PointCloudPtr cloud_src = slam_utils::convert3dPointsToCloud(last_spot_->getWorldPoints());

        // customtype::PointCloudPtr cloud_tgt = slam_utils::convert3dPointsToCloud(data_spot_ptr->getWorldPoints());

        // std::cout << cloud_src->size() << " " << std::cout << cloud_tgt->size() << std::endl;

        bool has_converged = false;
        double odom_variance;
        int odom_correspondences;
        // slam_utils::computeVariance(cloud_src, cloud_tgt, rel_transform, 10.0, &has_converged, &odom_variance, &odom_correspondences);

        double info = 100/(odom_drift_); //+ 2.0/data_spot_ptr->getId(); // before 1.0/(odom_variance*100);



        // Force 2D?
        // float x,y,z,r,p,yaw;
        //                     transform.getTranslationAndEulerAngles(x,y,z, r,p,yaw);
        //                     transform = Transform(x,y,0, 0, 0, yaw);
        //                     pcl::getTranslationAndEulerAngles(toEigen3f(), x, y, z, roll, pitch, yaw);

        std::cout << "ODOM: corr " <<  odom_correspondences << " info " << info << std::endl;

        DataLink3D::DataLinkPtr link( new DataLink3D() );
        link->inf_matrix_ = customtype::InformationMatrix3D::Identity();
        // std::cout << link->inf_matrix_ << std::endl;
        // before was 100
        if( info > 0 && info < 1000000 ) link->inf_matrix_ *= info;
        else if (info > 1000000) link->inf_matrix_ *= 1000000;//0.5;
        else link->inf_matrix_ *= 10;//0.5;

        // std::cout << "odometry infor matrix \n " << link->inf_matrix_ << std::endl;

        // before was 100
        // double odom_variance = 1;
        // double info = 1/odom_variance; // TEMPORARY -- TODO
        // // if( info > 0 && info < 1000000 ) link->inf_matrix_ *= info;
        // // else if (info > 1000000) link->inf_matrix_ *= 1000000;//0.5;
        // // else link->inf_matrix_ *= 10;//0.5;
        // link->inf_matrix_ *= info;

        link->transform_ = rel_transform;
        link->from_id_ = last_spot_->getId();
        link->to_id_ = data_spot_ptr->getId();
        link->active = true;
        link->type = DataLink3D::OdomConstraint;

        last_spot_->addLink(link);

        odom_drift_ += drift_rate_;

        if (data_spot_ptr->getId()%50==0)
            require_optimization_flag_ = true;
        // link->inf_matrix_; // Identity


    } // odometry constriant -- if()

    data_spots_.insert(data_spots_.end(), std::make_pair(data_spot_ptr->getId(),data_spot_ptr));
    last_spot_ = data_spot_ptr;
}

DataSpot3D::DataSpot3DPtr DataPool::getSpot(customtype::Identifier id){

    DataSpot3D::DataSpotMap::iterator it = data_spots_.find(id);
    if( it != data_spots_.end() )
        return it->second;
    else
        return DataSpot3D::DataSpot3DPtr();

}

int DataPool::getSize(){
    return data_spots_.size();
}

} // namespace gSlam