#include "VideoSource.h"
#include "highgui.h"
#include <iostream>

// namespace vo = visual_odometry;

int main(int argc, char** argv)
{

    VideoSource video_source;
    cv::Mat frame;
    std::string file_format = "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data4/image_%04d.png";
    std::string NEXT_FRAME_FMT = "/home/saif/msc_workspace/slam_test_bag_dataset/ardrone_flight_12_08_2017/data4/new/image_%04d.png";

    int findex = 0;
    while( !(frame = video_source.readNextFrame(file_format)).empty())
    {
        char buf[256];
        sprintf(buf, NEXT_FRAME_FMT.c_str(), findex);
        cv::imwrite(buf,frame);
        std::cout << findex << std::endl;
        findex+=1;
    }

    // std::cout << "Last image: " << buf << std::endl;
}