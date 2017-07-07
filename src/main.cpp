/** @file main.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */


#include "VideoSource.h"
#include "vis_odom.h"
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

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
    ros::NodeHandle n;
    // ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("vis_odom", 1000);
    tf::TransformBroadcaster odom_broadcaster;
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(1000);

    VideoSource video_source;
    cv::Mat frame;
    vo::VisOdom vOdom;
    std::stringstream traj_name;
    traj_name << "trajectory_scene" << argv[1] << ".txt";
    std::ofstream traj_out(traj_name.str());

    std::string path_prefix[] = { "S01_INPUT" , "S02_INPUT", "S03_INPUT"};
    std::string next_frame_format[] = { "S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png"};
    int i = 0;
    vOdom.init(video_source.readNextFrame(next_frame_format[SCENE-1]));

    visual_odometry::Frame::Ptr current_frame;
    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() && n.ok()){
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();
        current_frame = vOdom.process(frame,visualize_flag);


        // if( SCENE > 1 && i%300 == 0 )
        //     STAM.optimise();

        i++;
        cv::Mat p;

        cv::Mat pM = vOdom.intrinsics_*current_frame->projMatrix;//.mul(1.0/274759.971);
        for (int j = 0; j < 3; j++)
            traj_out << pM.at<double>(j, 0) << "," << pM.at<double>(j, 1) << "," << pM.at<double>(j, 2) << "," << pM.at<double>(j, 3) << std::endl;

        // TESTING ODOM PUBLISHING
        float x = 1.2; 
        float y = 3.2;
        float z = 34.2;
        float th = 53;

        // geometry_msgs::PoseStamped cam_pose;
        // cam_pose.header.stamp = current_time;
        // cam_pose.header.frame_id = "world_frame";
        // cam_pose.header.seq = i;
        // cam_pose.pose.position.x = current_frame->pose.at<double>(0,3);
        // cam_pose.pose.position.y = current_frame->pose.at<double>(1,3);
        // cam_pose.pose.position.z = current_frame->pose.at<double>(2,3);

        // double q1,q2,q3,q4;
        // current_frame->getQuaternion(q1,q2,q3,q4);
        // cam_pose.pose.orientation.x = q1;
        // cam_pose.pose.orientation.y = q2;
        // cam_pose.pose.orientation.z = q3;
        // cam_pose.pose.orientation.w = q4;

        // pose_pub.publish(cam_pose);
        // odom.header.stamp = current_time;
        // odom.header.frame_id = "odom";

        // odom.pose.pose.position.x = x;
        // odom.pose.pose.position.y = y;
        // odom.pose.pose.position.z = z;
        // odom.pose.pose.orientation = odom_quat;

        // odom_pub.publish(odom);
        int scale_ = (SCENE == 1)?1000:100;

        // TESTING ODOMETRY TRANSFORM BROADCASTING

        double q1,q2,q3,q4;
        current_frame->getQuaternion(q1,q2,q3,q4);
        // std::cout << q1 << " " << q2 << " " << q3 << " " << q4 << std::endl;

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "world_frame";
        odom_trans.child_frame_id = "cam_frame";

        odom_trans.transform.translation.x = -(current_frame->pose.at<double>(2,3))/scale_;
        odom_trans.transform.translation.y = (current_frame->pose.at<double>(0,3))/scale_;
        odom_trans.transform.translation.z = -(current_frame->pose.at<double>(1,3))/scale_;
        odom_trans.transform.rotation.x = q1;
        odom_trans.transform.rotation.y = q2;
        odom_trans.transform.rotation.z = q3;
        odom_trans.transform.rotation.w = q4;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        last_time = current_time;
        r.sleep();
    }

    vOdom.optimise();
    vOdom.dump();


    traj_out.close();

    printf("BYEBYE\n");

    return 0;
}
