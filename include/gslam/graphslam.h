#ifndef __GRAPH_SLAM__
#define __GRAPH_SLAM__

#include "gslam/typedefs.h"
#include "gslam/data_spot.h"
#include "gslam/data_pool.h"

namespace gSlam
{

class GrSLAM
{

public:

    typedef boost::shared_ptr<GrSLAM> Ptr;

    GrSLAM();
    ~GrSLAM();

    void processData(const customtype::TransformSE3& odom_pose,
                     const CameraParameters& cam_params,
                     const cv::Mat& image_color,
                     const customtype::ProjMatType& projectionMatrix);
                     // const cv::Mat& image_depth,
                     // const PointCloudPtr& cloud2d, const TimeStamp& stamp)

private:

    bool optimize_now_, optimize_near_, optimize_far_, keep_opt_thread_alive_;

    customtype::TransformSE3 map_correction_;

    customtype::Mutex mutex_graph_;
    customtype::CondVar cond_var_;

    DataPool data_pool_;



};// class GrSLAM

}//namespace gSlam


#endif // __GRAPH_SLAM__