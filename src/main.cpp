#include "STAM.h"
#include "VideoSource.h"
#include "gslam/graphslam.h"
#include "gslam/ros_visualizer.h"
#include <fstream>

namespace vo = visual_odometry;
bool visualize_flag = false;
bool ros_flag = false;
bool create_virtual_map = false;
int vis_odo_baseline = 100;
bool ismar_dataset = false;
int ideal_baselines_[] ={175, 50, 80, 
                        100, 100, 100, 75, 75, 
                        175, /*150*/
                        100, /*150*/ /*100 is probably better for loop closure*/ /*175*/
                        75, /*135*/ /*150*/
                        150,
                        130, /*130 is very good for 2 circles*//*100 works fine for 3 circles, but depth drift is more pronounced*/
                /*#14*/ 175, /* values from 100 to 300 seem to work fine */
                /*#15*/ 100, 
                /*#16*/ 100, 
                /*#17*/ 100, 
                /*#18*/ 100, 
                /*#19*/ 250, /* values from 100 to 300 seem to work fine. Over 200 is better. */
                /*#20*/ 250, /*300*/
                /*#21*/ 300, /*BAD DATASET*/
                /*#22*/ 150, /*125*/ /*175*/
                /*#23*/ 150};

bool write_file = false;
bool optimise_graph = false;

// Creating object for storing all parameterss that can be tuned. (Used for writing all the paramters used while writing trajectory to file)
namespace gSlam{ namespace SlamParameters   
    { SLAMinfo::SLAMinfoPtr info(new SLAMinfo); 
      const customtype::TransformSE3 pose_aligner_ = slam_utils::getFrameAligner(); // if no frame alignment required, use Identity
      const customtype::TransformSE3 ismar_frame_aligner_ = gSlam::slam_utils::getIsmarFrameAligner();
    } }


int main(int argc, char** argv)
{

    if( argc < 2 ){
        printf(" usage: rosrun graph_slam main_slam_node <scene_number> [visualize? (0/1)] [publish rostopics? (0/1)] [save trajectory to txt file? (0/1)] [run graph optimisation thread? (0/1)] [baseline for visual odometry]\n where <scene_number> = 1 - 23\n\n");
        exit(1);
    }
    else{
        SCENE = atoi(argv[1]);
        if( SCENE > 23 || SCENE < 1 )
        {
             printf(" usage: rosrun graph_slam main_slam_node <scene_number> [visualize? (0/1)] [publish rostopics? (0/1)] [save trajectory to txt file? (0/1)] [baseline for visual odometry]\n where <scene_number> = 1 - 23\n\n");
             exit(1);
        }


        if (ideal_baselines_[SCENE-1])
            vis_odo_baseline = ideal_baselines_[SCENE-1];
        // }

        if ( argc > 2 )
        {
            visualize_flag = (atoi(argv[2]) == 1); 
            std::cout << " visualizing " << std::endl;
        }
        if (argc > 3)
        {
            ros_flag = (atoi(argv[3]) == 1 || atoi(argv[3]) == 2);
            if (atoi(argv[3]) == 2)
                create_virtual_map = true;
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
        if (SCENE < 4)
        {
            optimise_graph = false;
            ismar_dataset = true;
        }
        std::cout << "Writing Trajectory: " << std::boolalpha << write_file << std::noboolalpha << std::endl;
        std::cout << "Running g2o graph optimisation: " << std::boolalpha << optimise_graph << std::noboolalpha << std::endl;
        std::cout << "Publishing ros messages: " << std::boolalpha << ros_flag << std::noboolalpha << std::endl;
        std::cout << "Creating virtual map: " << std::boolalpha << create_virtual_map << std::noboolalpha << std::endl;


    }


    std::string next_frame_format[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_2/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_3/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/data/image_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_2/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/data/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data1/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data2/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data3/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data4/image_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data5/image_%04d.png"};
    std::string intrinsics_file[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/intrinsicsS01.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/intrinsicsS02.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S03_INPUT/intrinsicsS03.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_1/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_3/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/intrinsics.xml", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/intrinsics.xml","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/intrinsics.xml"};
    std::string points3d_init_file[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/S01_3Ddata_dst_init.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/S02_3Ddata_dst_init.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S03_INPUT/S03_3Ddata_dst_init.csv","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/3dpoints.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/3dpoints.csv","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_2/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_checkerboard_22_07_17/ardrone_checkerboard_3/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/init_3Ddata.csv", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_circle_08_08_2-017/circle_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_1/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_2/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_3/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_4/init_3Ddata.csv","/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/ardrone_5/init_3Ddata.csv"};

    // ----- If using initWithCheckerboard method in STAM, the corresponding string should be "checkerboard"
    std::string template_file_fmt[] = {"/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S01_INPUT/S01L03_patch/S01L03_VGA_patch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/S02_INPUT/S02L03_patch/S02L03_VGA_patch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ismar/ S03_INPUT/S03L03_VGA_patch/S03L03_VGA_patch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb1/patches/ptch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/datasets_12_07_17/frontb2/patches/ptch_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square1/patches/ptch_%04d.png","/home/saif/msc_workspace/slam_test_bag_dataset/datasets_17_07_17/ardrone_front_square2/patches/ptch_%04d.png", "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_line_19_07_17/patches/ptch_%04d.png","checkerboard","checkerboard","checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard", "checkerboard"};
    

    // ROS Stuff ====================================================================================================

    ros::init(argc, argv, "Graph_Slam_Visualizer");
    gSlam::RosVisualizer visualizer(optimise_graph, ismar_dataset, create_virtual_map);

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

    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() && ros::ok())
    {
        try
        {
            // break; // ++++++++++++

            // ----- perform visual odometry on current frame
            current_odom_frame = vOdom.process(frame,visualize_flag);

            // ----- get 3D worldpoints for visualization in ROS. Only gets points when new features are tracked. For Visualization.
            gSlam::customtype::WorldPtsType world_points = vOdom.getNew3dPoints();

            // ----- gets 3D world points that are visible in each frame from STAM. For SLAM.
            gSlam::customtype::WorldPtsType points3d = vOdom.getCurrent3dPoints();
            // ----- get corresponding 2D image points from STAM.
            gSlam::customtype::KeyPoints key_points;
            cv::KeyPoint::convert(vOdom.getCurrent2dKeyPoints(), key_points);

            std::cout << world_points.size() << " " << points3d.size() << " " << key_points.size() << std::endl;

            // ----- get current camera pose from STAM 
            gSlam::customtype::TransformSE3 posemat;
            cv::cv2eigen(current_odom_frame->getCurrentPose(),posemat.matrix()); // conversion of cv::Mat to Eigen for quaternion calculation and further slam processing

            // ------- Align pose (in camera frame) with body frame of drone
            if (!ismar_dataset)
                posemat = posemat*gSlam::SlamParameters::pose_aligner_;
            else posemat = gSlam::SlamParameters::ismar_frame_aligner_*posemat;
            // else posemat = posemat*ismar_frame_aligner_;

            // gSlam::customtype::ProjMatType projectionMatrix;
            // cv::cv2eigen(current_odom_frame->projMatrix,projectionMatrix);

            // ----- Main slam processing 
            slam->processData(posemat, cam_params, frame, points3d, key_points);
            i++;

            // geometry_msgs::TransformStamped coordinate_correction = gSlam::ros_utils::setFrameCorrection(); // coordinate frame orientation correction for ISMAR dataset

            // ===== Creating and Publishing ROS Messages ===============================================================================
            if (ros_flag)
            {
             visualizer.updateRosMessagesAndPublish(world_points, slam->getDataPool().getDataSpots(), frame_no, posemat, key_points, frame, points3d);   
            }
            // ==========================================================================================================================

            frame_no++;
            // break;
        }

        catch (const cv::Exception& e)
        {
            std::cout << "Graph SLAM failed due to OpenCV Exception. Probably STAM failed due to correspondence loss. Try changing triangulation baseline.\n STOPPING Graph SLAM... " << std::endl;
            exit_safe = false;
            break;
        }
    } // while

    // ==================================================================================================================================

    gSlam::SlamParameters::info->frames_processed_ = frame_no;
    gSlam::SlamParameters::info->process_success_ = exit_safe;

    // ----- saving trajectory to txt file
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
    
    printf("EXITING... \n");

    std::cout << "SLAM PARAMETERS:-\n" << gSlam::slam_utils::getSlamParameterInfo(gSlam::SlamParameters::info) << "\n***\n";

    // delete slam;
    // delete vOdom;
    // delete visualizer;

    return 0;
}
