
#include "gslam/data_pool.h"
#include "gslam/fabmap.h"

namespace gSlam
{


DataPool::DataPool() : loop_count_far_(0), loop_count_near_(0) {}

void DataPool::addDataSpot(DataSpot3D::DataSpot3DPtr data_spot_ptr)

{
    std::cout << " Adding data spot " << std::endl;

    std::cout << " checking loop closure " << std::endl;

    //Loop closure constraints
    int new_id, loop_id;
    fabmap_.compareAndAdd(data_spot_ptr, new_id, loop_id);

    std::cout << " LoopID: " << loop_id << std::endl;

    // if( loop_id >= 0 && new_id > 0 ){}  // TODO: loop closure constraint

    std::cout << " Adding odometry " << std::endl;
    // Odometry constraint
    if( last_spot_.get() )
    {
        customtype::TransformSE3 rel_transform = last_spot_->getPose().inverse()*data_spot_ptr->getPose();
        float cx, cy, fx, fy;
        cx = last_spot_->getCamParams().cx_;
        cy = last_spot_->getCamParams().cy_;
        fx = last_spot_->getCamParams().fx_;
        fy = last_spot_->getCamParams().fy_;

        // std::cout << "ODOM: corr " <<  odom_correspondences << " info " << info << std::endl;

        // DataLink3D::DataLinkPtr link( new DataLink3D() );

        // // before was 100
        // if( info > 0 && info < 1000000 ) link->inf_matrix_ *= info;
        // else if (info > 1000000) link->inf_matrix_ *= 1000000;//0.5;
        // else link->inf_matrix_ *= 10;//0.5;

        // link->transform_ = rel_transform;
        // link->from_id_ = last_spot_->getId();
        // link->to_id_ = data_spot_ptr->getId();
        // link->active = true;
        // link->type = DataLink3D::OdomConstraint;

        // last_spot_->addLink(link);

        //link->inf_matrix_ // Identity


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