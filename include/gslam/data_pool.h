#ifndef __DATAPOOL__
#define __DATAPOOL__

#include "gslam/data_spot.h"
#include "gslam/fabmap.h"
#include "gslam/transform_estimator.h"

namespace gSlam
{

class DataPool {

public:

    typedef boost::shared_ptr<DataPool> DataPoolPtr;


    DataPool();


    void addDataSpot(DataSpot3D::DataSpot3DPtr data_spot_ptr);

    DataSpot3D::DataSpot3DPtr getSpot(customtype::Identifier id);

    DataSpot3D::DataSpot3DPtr getLastSpot(){ return last_spot_; }

    DataSpot3D::DataSpotMap& getDataSpots() { return data_spots_; }

    int getSize();

    void restartNewLoopsCountNear(){ new_count_loop_near_ = 0; }
    void restartNewLoopsCountFar(){ new_count_loop_far_ = 0; }


    int getNewLoopsCount() { return new_count_loop_far_ + new_count_loop_near_; }
    int getNewLoopsCountNear() { return  new_count_loop_near_; }
    int getNewLoopsCountFar() { return  new_count_loop_far_; }
    int getLoopCount() { return loop_count_far_ + loop_count_near_; }
    int getLoopCountNear() { return  loop_count_near_; }
    int getLoopCountFar() { return loop_count_far_; }

    bool checkOptimizationRequirement() { return require_optimization_flag_; }

private:
    DataSpot3D::DataSpot3DPtr last_spot_;
    DataSpot3D::DataSpotMap data_spots_;
    TransformEstimator transform_est_;

    FabMap fabmap_;

    int loop_count_far_; // far in time
    int loop_count_near_; // near in time

    int new_count_loop_far_; // far in time
    int new_count_loop_near_; // near in time

    int prev_loop_id_;
    bool repeat_match_;

    bool require_optimization_flag_;

};

}

#endif // __DATAPOOL__