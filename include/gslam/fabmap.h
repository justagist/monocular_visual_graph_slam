/** @file fabmap.h (Modification of Engel's FabMap implementation)
*
* @author  Jakob Engel <engelj at in dot tum dot de>
* @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
*
* @project graph_slam_rospkg
* @version 1.0
*
*/

/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#ifndef _CV_FAB_MAP_
#define _CV_FAB_MAP_

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <fstream>

#include <gslam/data_spot.h>


namespace gSlam{


///
/// Nearly the same usage as in LSD-SLAM (Jakob Engel)
///
class FabMap
{
public:

    FabMap();

    ~FabMap();


    /** Combination of compare() followed by add() (more efficient). */
    void compareAndAdd(const cv::Mat& keyFrameImage, int& out_newID, int& out_loopID, std::vector<cv::KeyPoint>& out_kpts);
    void compareAndAdd(DataSpot3D::DataSpot3DPtr data_spot_ptr, int& out_newID, int& out_loopID);

    /** Returns if the class is initialized correctly (i.e. if the required
     *  files could be loaded). */
    bool isValid() const;

private:
    int nextImageID;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::BOWImgDescriptorExtractor> bide;
    cv::Ptr<cv::of2::FabMap> fabMap;

    std::vector<cv::Mat> bow_storage_;
    int storage_retrival_counter_;

    int min_fabmap_baseline_, skip, first_bow_img_;

    cv::Mat prev_bow_;

    bool printConfusionMatrix;
    cv::Mat confusionMat;

    bool valid;

}; // class FabMap

}; // namespace



#endif //_CV_FAB_MAP_