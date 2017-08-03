#ifndef __SLAM_PARAMS__
#define __SLAM_PARAMS__

// #include "gslam/typedefs.h"

namespace gSlam
{

namespace Parameters
{
    class SLAMinfo
    {

    public:

    //     SLAMinfo(): dataset_id_(0), visual_odometry_baseline_(0), optimisation_thread_on_(false), odometry_info_const1(0), odometry_info_const2(0), loopclosure_info_const1(0), loopclosure_info_const2(0){}
        // ~SLAMinfo()

        // typedef boost::shared_ptr<SLAMinfo> SLAMinfoPtr;

        static int dataset_id_; // dataset used

        // int visual_odometry_baseline_; // triangulation baseline used in STAM visual odometry

        // bool optimisation_thread_on_; // whether optimisation thread was activated

        // float odometry_info_const1, odometry_info_const2; // the two constants in the formula for calculating information of odometry constraint: (1*a)/(drift*b)

        // float loopclosure_info_const1, loopclosure_info_const2; // the two constants in the formula for calculating information of loop closure constraint: (1*a)/(variance*b)





    }; // class SLAMinfo



    // static gSlam::Parameters::SLAMinfo::SLAMinfoPtr info(new gSlam::Parameters::SLAMinfo());
    // static gSlam::Parameters::SLAMinfo info;

} // namespace Constants

}// namesspace gSlam



#endif // __SLAM_PARAMS__