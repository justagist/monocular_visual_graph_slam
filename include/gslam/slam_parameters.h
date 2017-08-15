#ifndef __SLAM_PARAMS__
#define __SLAM_PARAMS__

// #include "gslam/typedefs.h"
// #include "gslam/slam_utils.h"

namespace gSlam
{

namespace SlamParameters
{
    class SLAMinfo
    {

    private:

        struct FabmapConstants
        {
            unsigned int first_bow_img_;
            unsigned int skip_;
        };

        struct InformationMatrixConstants
        {
            float const1_, const2_; // the two constants in the formula for calculating information of odometry constraint: (1*a)/(drift*b)
                              // the two constants in the formula for calculating information of loop closure constraint: (1*a)/ (variance*b) 
        };

        struct icpParameters
        {
            double inlier_threshold_, refine_sigma_;
            int max_iterations_, refine_max_iterations_;
            bool parameters_defined_ = false;
        };

        struct opticalFlowThresholds
        {
            float max_correspondence_error_;
            int min_correspondences_required_;
            int max_correspondence_used_;
            bool parameters_defined_ = false;
        };
        

    public:

        typedef boost::shared_ptr<SLAMinfo> SLAMinfoPtr;

        int frames_processed_;
        bool process_success_;

        int matcher_min_repetition_; // minimum times fabmap should detect the same frame consecutively as a loop closure to accept as true loop closure in DataSpotMatcher::findMatchingWorldPoints()
        int matcher_max_repetition_; // maximum repetition after which the counter is restarted


        int dataset_id_; // dataset used

        int visual_odometry_baseline_; // triangulation baseline used in STAM visual odometry

        bool optimisation_thread_on_; // whether optimisation thread was activated

        InformationMatrixConstants loopclosure_constraint, odometry_constraint;
        
        FabmapConstants fabmap;

        icpParameters transform_est_icp;

        opticalFlowThresholds lk_parameters;

    }; // class SLAMinfo



    // static gSlam::Parameters::SLAMinfo::SLAMinfoPtr info(new gSlam::Parameters::SLAMinfo());
    // static gSlam::Parameters::SLAMinfo info;
    extern SLAMinfo::SLAMinfoPtr info;

    // ----- Frame Alignement Transformation to align camera frame of the Ardrone to its body frame
    extern const Eigen::Affine3d pose_aligner_;
    extern const Eigen::Affine3d ismar_frame_aligner_;

} // namespace Constants

}// namesspace gSlam



#endif // __SLAM_PARAMS__