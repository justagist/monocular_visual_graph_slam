

#include "gslam/graphslam.h"

namespace gSlam
{

GrSLAM::GrSLAM() : map_correction_(customtype::TransformSE3::Identity()), optimize_now_(false), keep_opt_thread_alive_(true), optimize_near_(false), optimize_far_(false) {}

GrSLAM::~GrSLAM(){


    // Quit optimization thread

    {
        customtype::Lock lk(mutex_graph_);//*
        keep_opt_thread_alive_ = false;//*
        lk.unlock();//*

    }

    cond_var_.notify_all();//*




}

void GrSLAM::init(){ 
    optimize_now_ = false;
    keep_opt_thread_alive_ = true;
    optimize_graph_thread_ = std::thread(std::bind(&GrSLAM::optmizeGraphThread, this)); 
} 

// TODO: DEFINE DESTRUCTOR

void GrSLAM::processData(const customtype::TransformSE3& odom_pose,
                         const CameraParameters& cam_params,
                         const cv::Mat& image_color,
                         const customtype::ProjMatType& projectionMatrix,
                         const customtype::WorldPtsType& world_pts,
                         const customtype::KeyPoints& img_pts) // TIMESTAMP?!
{

    mutex_graph_.lock();
    customtype::TransformSE3 corrected_pose = map_correction_*odom_pose;
    mutex_graph_.unlock();
    // std::cout << "here!" << std::endl;  
    // std::cout << "here size " << world_pts.size() << std::endl;
    // std::cout << "here size " << img_pts.size() << std::endl;

    DataSpot3D::DataSpot3DPtr data_spot_new( new DataSpot3D(corrected_pose, cam_params, image_color, projectionMatrix, world_pts, img_pts));// tstamp));


    bool need_optimization = false;
    bool optimize_near = false;
    bool optimize_far = false;

    {
        customtype::Lock lk(mutex_graph_);

        data_pool_.addDataSpot(data_spot_new);
    // std::cout << " check !!" << data_spot_new->getKeyPoints().size() << std::endl;

        std::cout << " ADDED DATA SPOT " << std::endl;

        optimize_near = data_pool_.getNewLoopsCountNear() >= 2;
        // std::cout << "optimize near " << optimize_near << std::endl;
        optimize_far = data_pool_.getNewLoopsCountFar() >= 10; // before was 5
        need_optimization = optimize_near && optimize_far;
        need_optimization = optimize_near;

    }

    if( need_optimization )
    {
        GrSLAM::optimizeGraph(optimize_near, optimize_far );
    }


}

void GrSLAM::optimizeGraph(bool optimize_near, bool optimize_far){

    {
        customtype::Lock lk(mutex_graph_);
        std::cout << "reaching here " << std::endl;
        optimize_now_ = true;
        optimize_near_ = optimize_near;
        optimize_far_ = optimize_far;
        // Manual unlocking is done before notifying, to avoid waking up
        // the waiting thread only to block again (see notify_one for details)
        lk.unlock();

    }

    cond_var_.notify_one();


}


void GrSLAM::saveTrajectory(const std::string& filename){
    //Lock graph
    customtype::Lock* lock = NULL;
    lock = new customtype::Lock(this->getMutex());

    //Lock lock(mutex_graph_);
    std::ofstream output(filename);
    // Trajectory so far
    DataSpot3D::DataSpotMap& data_spots = this->getDataPool().getDataSpots();
    for(auto it = data_spots.begin(); it != data_spots.end(); it++)
    {
        customtype::TransformSE3 pose = it->second->getPose();
        // const customtype::TimeStamp& timestamp = it->second->getTimeStamp();
        Eigen::Vector3d t = pose.translation();
        Eigen::Quaternion<double> q(pose.rotation());
        // Format: timestamp tx ty tx qx qy qz qw
        output << it->second->getId() << " " << t.x() << " " <<  t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() <<  std::endl;


    }

    output.close();

    delete lock;
}

void GrSLAM::optmizeGraphThread(){

    while(keep_opt_thread_alive_){


        // Wait until main() sends data
        customtype::Lock lk(mutex_graph_);
        std::cout << "Worker is waiting\n" << std::endl;
        cond_var_.wait(lk, [&]{return optimize_now_;});
        std::cout << "Worker woke up\n" << std::endl;

        // If we need to quit, we quit.
        if( !keep_opt_thread_alive_ )
            break;

        // Setting graph
        pose_graph_.setVertices(data_pool_.getDataSpots());

        // after the wait, we own the lock.
        std::cout << "Worker thread is processing optimizing graph\n";


        //
        // Important stuff being done here
        // ...
        customtype::TransformSE3 optimized_pose, old_pose;

        old_pose = data_pool_.getLastSpot()->getPose(); // Old pose before optimization

        pose_graph_.fixed_iter_ = false;
        if( optimize_near_ ){
            pose_graph_.optimizeGraph(10);
        }
        else if( optimize_far_ ) {
            pose_graph_.fixed_iter_ = true;
            pose_graph_.optimizeGraph(50);
        }
        pose_graph_.updateVertices(data_pool_.getDataSpots());
        pose_graph_.release();

        //Just setting optimize_now_ to false, since we just optimized the graph
        optimize_now_ = false;

        optimized_pose = data_pool_.getLastSpot()->getPose(); // Now that's the new optimized pose

        map_correction_ = (optimized_pose*old_pose.inverse())*map_correction_;

        std::cout << "Map Correction: " << map_correction_.matrix() << std:: endl;

        // Send data back to main()
        //processed = true;
        std::cout << "Worker thread signals data processing completed\n";

        // Restarting loop count
        if( optimize_near_ ){
            data_pool_.restartNewLoopsCountNear();
            optimize_near_ = false;
        }
        if( optimize_far_ ){
            data_pool_.restartNewLoopsCountFar();
            optimize_far_ = false;
        }

        // Manual unlocking is done before notifying, to avoid waking up
        // the waiting thread only to block again (see notify_one for details)
        lk.unlock();
        cond_var_.notify_one();
    }

}


} // namespace gSlam