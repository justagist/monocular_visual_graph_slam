
#include "VideoSource.h"
#include <fstream>
#include "gslam/ros_utils.h"
#include "gslam/graphslam.h"

// #include <Eigen/Geometry>
// #include <Eigen/Dense>
// #include <opencv2/core/eigen.hpp>

namespace vo = visual_odometry;
bool visualize_flag = false;
bool ros_flag = false;
int main(int argc, char** argv){

    if( argc < 2 ){
        printf(" usage: rosrun visual_odom <node_name> <scene_number> [visualize? (1 = true | 0 = false(default))] [ros? (1 = true | 0 = false(default))]\n where <scene_number> = 1|2|3\n\n");
        exit(1);
    }
    else{
        SCENE = atoi(argv[1]);
        if( SCENE > 3 || SCENE < 1 )
        {
             printf(" usage: rosrun visual_odom <node_name> <scene_number> [visualize? (1 = true | 0 = false(default))] [ros? (1 = true | 0 = false(default))]\n where <scene_number> = 1|2|3\n\n");
             exit(1);
        }

        if ( argc > 2 )
        {
            int flag = atoi(argv[2]);
            visualize_flag = (flag == 1); 
            std::cout << " visualizing " << std::endl;
        }
        if (argc > 3)
        {
            int tmp = atoi(argv[3]);
            ros_flag = (tmp == 1);
        }

    }
    
    // ROS Stuff ====================================================================================================

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle rosNode;
    // ros::Publisher pose_pub = rosNode.advertise<geometry_msgs::PoseStamped>("vis_odom", 1000); // for publishing camera pose as posestamped msg
    tf::TransformBroadcaster odom_broadcaster;
    // tf::TransformBroadcaster frame_corrector; // coordinate frame orientation correction for ISMAR dataset
    ros::Publisher world_point_pub = rosNode.advertise<visualization_msgs::Marker>("worldpoints", 10);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(1000);

    visualization_msgs::Marker world_visualizer;
    world_visualizer.header.frame_id = "ismar_frame";
    world_visualizer.type = visualization_msgs::Marker::POINTS;

    // ==============================================================================================================


    // Initialising Visual Odometry using STAM ======================================================================

    VideoSource video_source;
    cv::Mat frame;
    vo::STAM vOdom;

    /* FOR TRAJECTORY OUTPUT
    **
    // std::stringstream traj_name;
    // traj_name << "trajectory_scene" << argv[1] << ".txt";
    // std::ofstream traj_out(traj_name.str());
    */

    std::string path_prefix[] = { "S01_INPUT" , "S02_INPUT", "S03_INPUT"};
    std::string next_frame_format[] = { "S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png"};
    int i = 0;
    vOdom.init(video_source.readNextFrame(next_frame_format[SCENE-1]));
    visual_odometry::Frame::Ptr current_odom_frame;
    gSlam::CameraParameters cam_params(vOdom.intrinsics_);
    // std::cout << vOdom.intrinsics_ << std::endl;
    // std::cout << cam_params.intrinsicsMat_.matrix() << std::endl;

    // ==============================================================================================================

    gSlam::GrSLAM::Ptr slam(new gSlam::GrSLAM());

    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() && rosNode.ok())

    {

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        // perform visual odometry on current frame
        current_odom_frame = vOdom.process(frame,visualize_flag);
        // std::cout << vOdom.key_frames_.size() << " size of keyframes " << std::endl;

        // cv::KeyPoint pt = current_odom_frame->keypoints.at(0);


        // Available methods from STAM: ------------------------------------------------------------------------------

        // gSlam::customtype::p2d_vec kpts = current_odom_frame->keypoints; // list of 2d keypoints in each frame
        // cv::Mat descriptors = current_odom_frame->descriptors; // list of descripts, one to each keypoint in the list of keypoints
        // gSlam::customtype::ProjectionCorrespondences kps = vOdom.getKeypointsInFrame(i); // get correspondence keypoints (3d and 2d) from STAM 

        // -----------------------------------------------------------------------------------------------------------

        // get 3D worldpoints for visualization in ROS
        gSlam::customtype::WorldPtsType world_points = vOdom.getCurrent3dPoints();
        // std::cout << "here size " << world_points.size() << std::endl;
        // for (int i = 0; i<world_points.size(); ++i)
        // {
        //     std::cout << world_points.at(i).x << " " << world_points.at(i).y << std::endl;
        // }

        gSlam::customtype::KeyPoints key_points;
        cv::KeyPoint::convert(vOdom.getCurrent2dKeyPoints(), key_points);
        // std::cout << "here size " << key_points.size() << std::endl;
        // for (int i = 0; i < key_points.size(); ++i)
        // {
        //     std::cout << key_points.at(i).pt.x << "  " << key_points.at(i).pt.y << std::endl;
        // }
        // std::cout << key_points.size() << " " << world_points.size() << std::endl;
        // std::cout << key_points << std::endl << world_points << std::endl;
        if (world_points.size()>0)
            gSlam::ros_utils::createPointMsg(world_visualizer, world_points);

        // get current camera pose from STAM
        gSlam::customtype::TransformSE3 posemat; 
        cv::cv2eigen(current_odom_frame->getCurrentPose(),posemat.matrix()); // conversion of cv::Mat to Eigen for quaternion calculation and further slam process

        gSlam::customtype::ProjMatType projectionMatrix;
        cv::cv2eigen(current_odom_frame->projMatrix,projectionMatrix);
        // std::cout << cam_params.intrinsicsMat_*projectionMatrix << std::endl;

        // std::cout << projectionMatrix << std::endl;
        // std::cout << projectionMatrix.block(0,0,3,3).inverse() << std::endl;

        slam->processData(posemat, cam_params, frame, projectionMatrix, world_points, key_points);
        /* STAM Bundle Adjustment 
        **
        // if( SCENE > 1 && i%300 == 0 )
        //     STAM.optimise();
        */

        i++;
        // cv::Mat p;
        // std::cout << "eigen: " << cam_params.intrinsicsMat_*prj << std::endl;
         // writing trajectory to file
        /**/
        // cv::Mat pM = vOdom.intrinsics_*current_odom_frame->projMatrix;//.mul(1.0/274759.971);
        // std::cout << " cv: " << pM << std::endl;
        // for (int j = 0; j < 3; j++)
        //     traj_out << pM.at<double>(j, 0) << "," << pM.at<double>(j, 1) << "," << pM.at<double>(j, 2) << "," << pM.at<double>(j, 3) << std::endl;
        

        
        geometry_msgs::TransformStamped odom_trans = gSlam::ros_utils::createOdomMsg(posemat);

        // geometry_msgs::TransformStamped coordinate_correction = gSlam::ros_utils::setFrameCorrection(); // coordinate frame orientation correction for ISMAR dataset
        if (ros_flag)
        {
        //publish the transform and world points
        odom_broadcaster.sendTransform(odom_trans);
        world_point_pub.publish(world_visualizer);
        // frame_corrector.sendTransform(coordinate_correction); // coordinate frame orientation correction for ISMAR dataset
        }


        last_time = current_time;
        r.sleep();
        break;

    } // while

    // vOdom.optimise();
    // vOdom.dump();


    // traj_out.close();

    printf("EXITING\n");

    return 0;
}
