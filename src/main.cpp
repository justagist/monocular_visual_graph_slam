
#include "VideoSource.h"
#include <fstream>
#include "gslam/ros_utils.h"

// #include <Eigen/Geometry>
// #include <Eigen/Dense>
// #include <opencv2/core/eigen.hpp>

namespace vo = visual_odometry;
bool visualize_flag;

int main(int argc, char** argv){

    if( argc < 2 ){
        printf(" usage: ./stam <scene_number>\n where <scene_number> = 1|2|3\n\n");
        exit(1);
    }
    else{
        SCENE = atoi(argv[1]);
        if( SCENE > 3 || SCENE < 1 )
        {
             printf(" usage: ./stam <scene_number>\n where <scene_number> = 1|2|3\n\n");
             exit(1);
        }

        if ( argc == 3 )
        {
            int flag = atoi(argv[2]);
            visualize_flag = (flag == 1); 
        }
        else visualize_flag = false;


    }
    
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle rosNode;
    // ros::Publisher pose_pub = rosNode.advertise<geometry_msgs::PoseStamped>("vis_odom", 1000);
    tf::TransformBroadcaster odom_broadcaster;
    // tf::TransformBroadcaster frame_corrector; // coordinate frame orientation correction for ISMAR dataset
    ros::Publisher world_point_pub = rosNode.advertise<visualization_msgs::Marker>("worldpoints", 10);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(1000);

    VideoSource video_source;
    cv::Mat frame;
    vo::STAM vOdom;
    // std::stringstream traj_name;
    // traj_name << "trajectory_scene" << argv[1] << ".txt";
    // std::ofstream traj_out(traj_name.str());

    std::string path_prefix[] = { "S01_INPUT" , "S02_INPUT", "S03_INPUT"};
    std::string next_frame_format[] = { "S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png"};
    int i = 0;
    vOdom.init(video_source.readNextFrame(next_frame_format[SCENE-1]));

    visual_odometry::Frame::Ptr current_odom_frame;

    // gSlam::customtype::PointCloudPtr cloud_msg (new gSlam::customtype::PointCloud);
    // cloud_msg->header.frame_id = "ismar_frame";
    // cloud_msg->height = cloud_msg->width = 1;
    visualization_msgs::Marker world_visualizer;
    world_visualizer.header.frame_id = "ismar_frame";
    world_visualizer.type = visualization_msgs::Marker::POINTS;

    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() && rosNode.ok())

    {

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        // perform visual odometry on current frame
        current_odom_frame = vOdom.process(frame,visualize_flag);

        // cv::KeyPoint pt = current_odom_frame->keypoints.at(0);
        // std::cout << current_odom_frame->keypoints.at(0).pt << std::endl;

        gSlam::customtype::p2d_vec img_pts = current_odom_frame->keypoints;

        // get correspondence keypoints (3d and 2d) from STAM 
        // gSlam::customtype::ProjectionCorrespondences kps = vOdom.getKeypointsInFrame(i);

        // if( SCENE > 1 && i%300 == 0 )
        //     STAM.optimise();

        std::vector<cv::Point3d> world_points = vOdom.getCurrent3dPoints();
        if (world_points.size()>0)
            gSlam::ros_utils::createPointMsg(world_visualizer, world_points);


        i++;
        cv::Mat p;

         /*writing trajectory to file
        **
        // cv::Mat pM = vOdom.intrinsics_*current_odom_frame->projMatrix;//.mul(1.0/274759.971);
        // for (int j = 0; j < 3; j++)
        //     traj_out << pM.at<double>(j, 0) << "," << pM.at<double>(j, 1) << "," << pM.at<double>(j, 2) << "," << pM.at<double>(j, 3) << std::endl;
        */

        // get current camera pose from STAM
        gSlam::customtype::TransformSE3 posemat; 
        cv::cv2eigen(current_odom_frame->getCurrentPose(),posemat.matrix());
        
        geometry_msgs::TransformStamped odom_trans = gSlam::ros_utils::createOdomMsg(posemat);
        // geometry_msgs::TransformStamped coordinate_correction = gSlam::ros_utils::setFrameCorrection(); // coordinate frame orientation correction for ISMAR dataset

        //publish the transform
        odom_broadcaster.sendTransform(odom_trans);
        world_point_pub.publish(world_visualizer);

        // pointPub.publish (cloud_msg);
        // frame_corrector.sendTransform(coordinate_correction);

        last_time = current_time;
        r.sleep();
        // break;

    } // while

    // vOdom.optimise();
    // vOdom.dump();


    // traj_out.close();

    printf("BYEBYE\n");

    return 0;
}
