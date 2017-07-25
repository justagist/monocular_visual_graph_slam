
#include "gslam/data_pool.h"
#include "gslam/fabmap.h"

namespace gSlam
{


DataPool::DataPool() : loop_count_far_(0), loop_count_near_(0) {}

void DataPool::addDataSpot(DataSpot3D::DataSpot3DPtr data_spot_ptr)

{
    // std::cout << " Adding data spot " << std::endl;

    // std::cout << " checking loop closure " << std::endl;

    //Loop closure constraints
    int new_id, loop_id;
    fabmap_.compareAndAdd(data_spot_ptr, new_id, loop_id);
    // std::cout << data_spot_ptr->getImagePoints().size() << "check " << std::endl;
    // std::cout << data_spot_ptr->getWorldPoints().size() << " check" << std::endl;
    assert(data_spot_ptr->getImagePoints().size() == data_spot_ptr->getWorldPoints().size());

    std::cout << " LoopID: " << loop_id << " new_id: " << new_id << std::endl;

    if( loop_id >= 0 && new_id > 0 && (new_id - loop_id) > 50) 
    {
        std::cout << "Possible loop closure : " << new_id << "->" << loop_id << std::endl;
        double variance, prop_matches;
        int correspondences;
        bool status_good = false;
        DataLink3D::DataLinkPtr link( new DataLink3D() );

        DataSpot3D::DataSpot3DPtr spot_src = data_spots_.find(loop_id)->second;

        link->from_id_ = spot_src->getId();
        link->to_id_ = data_spot_ptr->getId();
        
        std::cout << " Estimating Loop Closure Transform " <<std::endl;
        link->transform_ = transform_est_.estimateTransform(spot_src, data_spot_ptr, variance, correspondences, prop_matches, status_good);

        // --- Enforce 2D ---
        // float x,y,z,r,p,yaw;
        // slam_x_utils::getTranslationAndEulerAngles(link->transform_, x, y, z, r, p , yaw);
        // link->transform_ = slam_utils::getTransformation(x, y, 0, 0, 0, yaw);

        if (variance == 0)
            variance = 1.0;
        double info = 1.0/(variance); // before 1.0/(variance*100);

        // info before was 100
        // if( info > 0 && info < 1000000) link->inf_matrix_ *= info*1;
        // else link->inf_matrix_ *= 1;//0.5*(1+prop_matches)*2;
        link->inf_matrix_ *= info;
        link->active = true;
        link->type = DataLink3D::LoopClosureConstraint;
        std::cout << "LOOP: corr " <<  correspondences << /*" info " << info <<*/ std::endl;
        if( status_good )
        {

            bool valid = true;
            if( (new_id - loop_id) > 10 ) // FAR LOOP
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

            if( dist > 0.20 ) // If near loop is close in time but too far to be near, then it's not a valid transform
            { 
                valid = false;
                // std::cout << " too far to be a near loop: dist: " <<dist << std::endl;
            }

            if( valid ) 
            {
                spot_src->addLink(link);

                std::cout << " LOOP ADDED ! NFar " << loop_count_far_ << " NNear " << loop_count_near_ << std::endl;
            }
        }
    }  // loop closure -- if()

    // std::cout << " Adding odometry " << std::endl;
    // Odometry constraint
    if( last_spot_.get() )
    {
        customtype::TransformSE3 rel_transform = last_spot_->getPose().inverse()*data_spot_ptr->getPose();
        // float cx, cy, fx, fy;
        // cx = last_spot_->getCamParams().cx_;
        // cy = last_spot_->getCamParams().cy_;
        // fx = last_spot_->getCamParams().fx_;
        // fy = last_spot_->getCamParams().fy_;

        // std::cout << "ODOM: corr " <<  odom_correspondences << " info " << info << std::endl;

        DataLink3D::DataLinkPtr link( new DataLink3D() );


        // before was 100
        double odom_variance = 1;
        double info = 1/odom_variance; // TEMPORARY -- TODO
        // if( info > 0 && info < 1000000 ) link->inf_matrix_ *= info;
        // else if (info > 1000000) link->inf_matrix_ *= 1000000;//0.5;
        // else link->inf_matrix_ *= 10;//0.5;
        link->inf_matrix_ *= info;

        link->transform_ = rel_transform;
        link->from_id_ = last_spot_->getId();
        link->to_id_ = data_spot_ptr->getId();
        link->active = true;
        link->type = DataLink3D::OdomConstraint;

        last_spot_->addLink(link);

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