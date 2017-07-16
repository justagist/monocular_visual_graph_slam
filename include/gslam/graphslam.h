#ifndef __GRAPH_SLAM__
#define __GRAPH_SLAM__

#include "gslam/typedefs.h"
#include "gslam/data_spot.h"
#include "gslam/data_pool.h"
#include "gslam/graph.h"

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
                     const customtype::ProjMatType& projectionMatrix,
                     const customtype::WorldPtsType& world_pts,
                     const customtype::KeyPoints& img_pts);
                     // const cv::Mat& image_depth,
                     // const PointCloudPtr& cloud2d, const TimeStamp& stamp)

    void init(); //*

    void optimizeGraph(bool optimize_near, bool optimize_far); // Signals a separate thread (optmizeGraphThread) for running graph optimization
    void optmizeGraphThread(); // Does the actual graph optimization
    void saveTrajectory(const std::string& filename);
    customtype::Mutex& getMutex() { return mutex_graph_; }
    DataPool& getDataPool() { return data_pool_; }

    // Gets map correction atomically
    customtype::TransformSE3 getMapCorrection() {
        mutex_graph_.lock();
        customtype::TransformSE3 map_correction = map_correction_;
        mutex_graph_.unlock();
        return map_correction;
    }

private:

    bool optimize_now_, optimize_near_, optimize_far_, keep_opt_thread_alive_;

    customtype::TransformSE3 map_correction_;

    customtype::Mutex mutex_graph_;
    customtype::CondVar cond_var_;

    DataPool data_pool_;

    GraphOptimizer pose_graph_;

    std::thread optimize_graph_thread_;

};// class GrSLAM

}//namespace gSlam


#endif // __GRAPH_SLAM__