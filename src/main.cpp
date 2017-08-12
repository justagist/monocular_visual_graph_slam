#include "VideoSource.h"
#include <fstream>
#include "gslam/ros_utils.h"
#include "gslam/graphslam.h"

namespace vo = visual_odometry;
bool visualize_flag = false;
bool ros_flag = false;
int vis_odo_baseline = 100;
int ismar_baselines[] ={175, 50, 80, 
                        100, 100, 100, 75, 75, 
                        175, /*150*/
                        100, /*150*/ /*100 is probably better for loop closure*/ /*175*/
                        75, /*135*/ /*150*/
                        150,
                        130 /*130 is very good for 2 circles*//*100 works fine for 3 circles, but depth drift is more pronounced*/};
bool write_file = false;
bool optimise_graph = false;

// Creating object for storing all parameterss that can be tuned. (Used for writing all the paramters used while writing trajectory to file)
namespace gSlam{ namespace SlamParameters   
    { SLAMinfo::SLAMinfoPtr info(new SLAMinfo); 
      const customtype::TransformSE3 pose_aligner_ = slam_utils::getFrameAligner(); // if no frame alignment required, use Identity
    } }


int main(int argc, char** argv)
{

    if( argc < 2 ){
        printf(" usage: rosrun graph_slam main_slam_node <scene_number> [visualize? (0/1)] [publish rostopics? (0/1)] [save trajectory to txt file? (0/1)] [run graph optimisation thread? (0/1)] [baseline for visual odometry]\n where <scene_number> = 1 - 18\n\n");
        exit(1);
    }
    else{
        SCENE = atoi(argv[1]);
        if( SCENE > 18 || SCENE < 1 )
        {
             printf(" usage: rosrun graph_slam main_slam_node <scene_number> [visualize? (0/1)] [publish rostopics? (0/1)] [save trajectory to txt file? (0/1)] [baseline for visual odometry]\n where <scene_number> = 1 - 18\n\n");
             exit(1);
        }

        if (ismar_baselines[SCENE-1])
            vis_odo_baseline = ismar_baselines[SCENE-1];
        // }

        if ( argc > 2 )
        {
            visualize_flag = (atoi(argv[2]) == 1); 
            std::cout << " visualizing " << std::endl;
        }
        if (argc > 3)
        {
            ros_flag = (atoi(argv[3]) == 1);
        }
        if (argc > 4)
        {
            write_file = (atoi(argv[4])==1);
        }
        if (argc > 5)
        {
            optimise_graph = (atoi(argv[5]) == 1);
        }
        if (argc > 6)
        {
            if (atoi(argv[6])!=0)
                vis_odo_baseline = atoi(argv[6]);
        }
        std::cout << "Writing Trajectory: " << std::boolalpha << write_file << std::noboolalpha << std::endl;
        std::cout << "Running g2o graph optimisation: " << std::boolalpha << optimise_graph << std::noboolalpha << std::endl;
        std::cout << "Publishing ros messages: " << std::boolalpha << ros_flag << std::noboolalpha << std::endl;

    }


    std::string next_frame_format[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_2/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_3/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_2/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/data/image_%04d.png"};
    std::string intrinsics_file[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/intrinsicsS01.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/intrinsicsS02.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S03_INPUT/intrinsicsS03.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_1/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_3/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/intrinsics.xml"};
    std::string points3d_init_file[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/S01_3Ddata_dst_init.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/S02_3Ddata_dst_init.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S03_INPUT/S03_3Ddata_dst_init.csv","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/3dpoints.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/3dpoints.csv","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_2/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_3/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/init_3Ddata.csv"};

    // ----- If using initWithCheckerboard method in STAM, the corresponding string should be "checkerboard"
    std::string template_file_fmt[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/S01L03_patch/S01L03_VGA_patch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/S02L03_patch/S02L03_VGA_patch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/ S03_INPUT/S03L03_VGA_patch/S03L03_VGA_patch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/patches/ptch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/patches/ptch_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/patches/ptch_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/patches/ptch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/patches/ptch_%04d.png","checkerboard","checkerboard","checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard"};
    

    // ROS Stuff ====================================================================================================

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle rosNode;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(1000);

    tf::TransformBroadcaster odom_broadcaster;
    // tf::TransformBroadcaster frame_corrector; // coordinate frame orientation correction for ISMAR dataset -- NOT DONE CORRECTLY YET.

    ros::Publisher marker_pub = rosNode.advertise<visualization_msgs::Marker>("markers", 10); // visualizing 3d worldpoints detected by STAM (can also be used for publishing (optimised) trajectory using markers).
    visualization_msgs::Marker world_visualizer, optimised_trajectory_msg, updated_worldpts_msg; // 'optimised_trajectory_msg' is used only if marker message is used for publishing trajectory.

    // ----- world_visualizer: for visualizing the world points as obtained from STAM 
    world_visualizer.header.frame_id = "world_frame";
    world_visualizer.ns = "3D Keypoints";
    world_visualizer.type = visualization_msgs::Marker::POINTS;
    world_visualizer.id = 0;
    world_visualizer.color.g = 1.0f;

    // ----- updated_worldpts_msg: visualizes the updated world points after graph optimisation
    updated_worldpts_msg.header.frame_id = "world_frame";
    updated_worldpts_msg.ns = "Updated 3D Keypoints";
    updated_worldpts_msg.type = visualization_msgs::Marker::POINTS;
    updated_worldpts_msg.id = 2;
    updated_worldpts_msg.color.r = 1.0f;

    // ----- Pulblisher for publishing trajectory as Path message. If using Marker message for publishing trajectory, this is not required.
    ros::Publisher trajectory_publisher = rosNode.advertise<nav_msgs::Path>("trajectory",1000);
    nav_msgs::Path path_msg;
    // ------------


    // ==============================================================================================================


    // Initialising Visual Odometry using STAM ======================================================================

    VideoSource video_source;
    cv::Mat frame;
    vo::STAM vOdom;

    int i = 0;

    // ----- Initialising STAM using checkerboard method (check in stam.cpp)
    if (template_file_fmt[SCENE-1] == "checkerboard")
    {
        std::cout << "Initialising STAM using checkerboard method" << std::endl;
        vOdom.init(video_source.readNextFrame(next_frame_format[SCENE-1]),next_frame_format[SCENE-1], intrinsics_file[SCENE-1], points3d_init_file[SCENE-1], vis_odo_baseline);    
    }
    // ----- Initialise from template files (image patches whose 3D positions in the world frame are known)
    else vOdom.init(video_source.readNextFrame(next_frame_format[SCENE-1]),next_frame_format[SCENE-1], intrinsics_file[SCENE-1], points3d_init_file[SCENE-1], template_file_fmt[SCENE-1], vis_odo_baseline);
    visual_odometry::Frame::Ptr current_odom_frame;

    // ----- Using camera parameters loaded by STAM
    gSlam::CameraParameters cam_params(vOdom.intrinsics_, vOdom.getDistortion());

    // ==============================================================================================================

    gSlam::GrSLAM::Ptr slam(new gSlam::GrSLAM());

    if (gSlam::SlamParameters::info)
        std::cout << "RECORDING SLAM PARAMETERS TO info OBJECT" << std::endl;
    else std::cout << "NO DEFINITION" << std::endl;

    // ----- Recording parameters
    gSlam::SlamParameters::info->optimisation_thread_on_ = optimise_graph;
    gSlam::SlamParameters::info->dataset_id_ = SCENE;
    gSlam::SlamParameters::info->visual_odometry_baseline_ = vis_odo_baseline;

    if (optimise_graph)
        slam->init();

    int frame_no = 0;

    bool exit_safe = true;

    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() && rosNode.ok())
    {
        try
        {
            // break; // ++++++++++++

            ros::spinOnce(); // check for incoming messages
            current_time = ros::Time::now();

            // ----- perform visual odometry on current frame
            current_odom_frame = vOdom.process(frame,visualize_flag);

            // ----- get 3D worldpoints for visualization in ROS. Only gets points when new features are tracked. For Visualization.
            gSlam::customtype::WorldPtsType world_points = vOdom.getNew3dPoints();

            // ----- gets 3D world points that are visible in each frame. For SLAM.
            gSlam::customtype::WorldPtsType points3d = vOdom.getCurrent3dPoints();

            gSlam::customtype::KeyPoints key_points;
            cv::KeyPoint::convert(vOdom.getCurrent2dKeyPoints(), key_points);

            // ----- get current camera pose from STAM in the required type
            gSlam::customtype::TransformSE3 posemat;
            cv::cv2eigen(current_odom_frame->getCurrentPose(),posemat.matrix()); // conversion of cv::Mat to Eigen for quaternion calculation and further slam processes

            // ------- Align pose (in camera frame) with body frame of drone
            posemat = posemat*gSlam::SlamParameters::pose_aligner_;

            // gSlam::customtype::ProjMatType projectionMatrix;
            // cv::cv2eigen(current_odom_frame->projMatrix,projectionMatrix);

            // ----- Main slam processing 
            slam->processData(posemat, cam_params, frame, points3d, key_points);
            i++;

            // geometry_msgs::TransformStamped coordinate_correction = gSlam::ros_utils::setFrameCorrection(); // coordinate frame orientation correction for ISMAR dataset

            // ===== Creating and Publishing ROS Messages ===============================================================================
            if (ros_flag)
            {
                // -------- update world_visualizer only when new world points are observed by STAM
                if (world_points.size()>0)
                {
                    gSlam::ros_utils::createPointMsg(world_points, world_visualizer);
                    if (optimise_graph)
                        gSlam::ros_utils::storeTruePose(frame_no, posemat);
                        gSlam::ros_utils::checkMapUpdateAndCreateNewPointMsg(slam->getDataPool().getDataSpots(), updated_worldpts_msg);
                }
                
                geometry_msgs::TransformStamped odom_trans = gSlam::ros_utils::createOdomMsg(posemat);

                //// ------ Use 1 of the following trajectory message types
                // optimised_trajectory_msg = gSlam::ros_utils::createOptimisedTrajectoryMsg(slam->getDataPool().getDataSpots());
                path_msg = gSlam::ros_utils::createPathMsg(slam->getDataPool().getDataSpots());
                //// --------------------------------------------

                // -------- publish the transform and world points
                odom_broadcaster.sendTransform(odom_trans);
                marker_pub.publish(world_visualizer);
                if (optimise_graph)
                    marker_pub.publish(updated_worldpts_msg);
                // marker_pub.publish(optimised_trajectory_msg); // for publishing trajectory using markers

                //// ------ Use only if publishing path message and not marker message for trajectory
                trajectory_publisher.publish(path_msg);
                //// ----------------------------
            }
            // ==========================================================================================================================

            last_time = current_time;
            r.sleep();

            frame_no++;
            // if (i==2)
            // break;
        }
        catch (const cv::Exception& e)
        {
            std::cout << "Graph SLAM failed due to OpenCV Exception. Probably STAM failed due to correspondence loss. Try changing triangulation baseline.\n STOPPING Graph SLAM... " << std::endl;
            exit_safe = false;
            break;
        }
    } // while

    gSlam::SlamParameters::info->frames_processed_ = frame_no;
    gSlam::SlamParameters::info->process_success_ = exit_safe;

    if (write_file)
    {
        std::stringstream traj_file;
        std::string traj_name;
        if (optimise_graph)
            traj_name = "optimised_trajectory";
        else traj_name = "trajectory";

        if (argc > 7)
            traj_name = argv[7];

        traj_file << "/home/saif/test_ws/src/graph_slam/estimated_trajectories/" << traj_name << "_" << SCENE << "_" << vis_odo_baseline <<".txt";

        if (slam->getDataPool().getDataSpots().size() > 1)
        {
            slam->saveTrajectory(traj_file.str());
            std::cout << "Wrote trajectory to file: " << traj_file.str() << std::endl;
        }
        else std::cout << "No poses were found! Trajectory file not written." << std::endl;
    }
    
    std::cout << "SLAM PARAMETERS:-\n" << gSlam::slam_utils::getSlamParameterInfo(gSlam::SlamParameters::info) << "\n***\n";

    printf("EXITING\n");

    return 0;
}
