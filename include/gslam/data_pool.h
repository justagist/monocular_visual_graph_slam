/** @file datapool.h (class containing all dataspots. checks for loop closures, creates loop closure constraints, odometry constraints)
*
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

#ifndef __DATAPOOL__
#define __DATAPOOL__

#include "gslam/data_spot.h"
#include "gslam/fabmap.h"
#include "gslam/transform_estimator.h"

namespace gSlam
{

class DataPool {

public:

    typedef std::shared_ptr<DataPool> DataPoolPtr;


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
    // bool repeat_match_;
    int repeat_match_count_;
    int min_required_repeat_, max_repeat_allowed_;

    bool require_optimization_flag_, loop_match_success_;

    float odom_drift_, drift_rate_;

};

}

#endif // __DATAPOOL__