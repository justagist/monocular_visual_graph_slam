#ifndef __DATASPOT__
#define __DATASPOT__

#include "gslam/typedefs.h"
namespace gSlam
{

class CameraParameters{
public:

    union {
        struct {
            float fx_, fy_, cx_, cy_;
        };

        float params_[4];

    };


    CameraParameters(float fx, float fy, float cx, float cy) {

        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;

    }

    CameraParameters(const cv::Mat& intrinsics_)
    {
        fx_ = intrinsics_.at<double>(0,0);
        fy_ = intrinsics_.at<double>(1,1);
        cx_ = intrinsics_.at<double>(0,2);
        cy_ = intrinsics_.at<double>(1,2);
    }

    CameraParameters(const CameraParameters& other) {
        fx_ = other.fx_;
        fy_ = other.fy_;
        cx_ = other.cx_;
        cy_ = other.cy_;
    }

    CameraParameters() {
        fx_ = fy_ = cx_ = cy_ = 0;
    }



}; // class CameraParameters

} // namespace gSlam

#endif // __DATASPOT__