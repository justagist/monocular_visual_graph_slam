/** @file main.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */


#include "VideoSource.h"
#include "STAM.h"
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace vo = visual_odometry;

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


    }

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("vis_odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(1000);

    VideoSource video_source;
    cv::Mat frame;
    vo::STAM STAM;
    std::stringstream traj_name;
    traj_name << "trajectory_scene" << argv[1] << ".txt";
    std::ofstream traj_out(traj_name.str());

    std::string path_prefix[] = { "S01_INPUT" , "S02_INPUT", "S03_INPUT"};
    std::string next_frame_format[] = { "S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png"};
    int i = 0;
    STAM.init(video_source.readNextFrame(next_frame_format[SCENE-1]));

    visual_odometry::Frame::Ptr current_frame;
    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() && n.ok()){
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();
        current_frame = STAM.process(frame);


        if( SCENE > 1 && i%300 == 0 )
            STAM.optimise();

        i++;
        cv::Mat p;

        cv::Mat pM = STAM.intrinsics_*current_frame->projMatrix;//.mul(1.0/274759.971);
        for (int j = 0; j < 3; j++)
            traj_out << pM.at<double>(j, 0) << "," << pM.at<double>(j, 1) << "," << pM.at<double>(j, 2) << "," << pM.at<double>(j, 3) << std::endl;

        // TESTING ODOM PUBLISHING
        float x = 1.2; 
        float y = 3.2;
        float z = 34.2;
        float th = 53;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        // nav_msgs::Odometry odom;
        // odom.header.stamp = current_time;
        // odom.header.frame_id = "odom";

        // odom.pose.pose.position.x = x;
        // odom.pose.pose.position.y = y;
        // odom.pose.pose.position.z = z;
        // odom.pose.pose.orientation = odom_quat;

        // odom_pub.publish(odom);

        // last_time = current_time;
        // r.sleep();

        // TESTING ODOMETRY TRANSFORM BROADCASTING

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

    }

    STAM.optimise();
    STAM.dump();


    traj_out.close();

    printf("BYEBYE\n");

    return 0;
}
