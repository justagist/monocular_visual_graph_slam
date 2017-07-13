

#include "gslam/graphslam.h"

namespace gSlam
{

GrSLAM::GrSLAM() : map_correction_(customtype::TransformSE3::Identity()), optimize_now_(false), keep_opt_thread_alive_(true), optimize_near_(false), optimize_far_(false) {}

GrSLAM::~GrSLAM(){}

// TODO: DEFINE DESTRUCTOR

void GrSLAM::processData(const customtype::TransformSE3& odom_pose,
                    const CameraParameters& cam_params,
                    const cv::Mat& image_color) // TIMESTAMP?!
{

    mutex_graph_.lock();
    customtype::TransformSE3 corrected_pose = map_correction_*odom_pose;
    mutex_graph_.unlock();
    std::cout << "here!" << std::endl;  

    DataSpot3D::DataSpot3DPtr data_spot_new( new DataSpot3D(corrected_pose, cam_params, image_color));// stamp));

    bool need_optimization = false;
    bool optimize_near = false;
    bool optimize_far = false;

    {
        customtype::Lock lk(mutex_graph_);

        data_pool_.addDataSpot(data_spot_new);

        // std::cout << " ADDED DATA SPOT " << std::endl;

        // optimize_near = data_pool_.getNewLoopsCountNear() >= 2;
        // optimize_far = data_pool_.getNewLoopsCountFar() >= 10; // before was 5
        // need_optimization = optimize_near && optimize_far;
        // need_optimization = optimize_near;

    }


}



} // namespace gSlam