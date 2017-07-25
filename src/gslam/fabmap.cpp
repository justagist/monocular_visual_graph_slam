/** @file fabmap.cpp
 *
 * @author  Jakob Engel <engelj at in dot tum dot de>
 * @author  Ermano A Arruda (eaa3@cin.ufpe.br)
 *
 * Copyright (c) 2015 Ermano A Arruda. All rights reserved.
 *
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


#include "gslam/fabmap.h"



namespace gSlam
{

// FabMap::FabMap()
// {
//     valid = false;

//     std::string fabmapTrainDataPath = "/home/saif/msc_workspace/slam_x_lib/training_data/StLuciaShortTraindata.yml";
//     std::string vocabPath =  "/home/saif/msc_workspace/slam_x_lib/training_data/StLuciaShortVocabulary.yml";
//     std::string chowliutreePath =  "/home/saif/msc_workspace/slam_x_lib/training_data/StLuciaShortTree.yml";

//     // Load training data
//     cv::FileStorage fsTraining;
//     fsTraining.open(fabmapTrainDataPath, cv::FileStorage::READ);
//     cv::Mat fabmapTrainData;
//     fsTraining["BOWImageDescs"] >> fabmapTrainData;
//     if (fabmapTrainData.empty()) {
//         std::cerr << fabmapTrainDataPath << ": FabMap Training Data not found" << std::endl;
//         return;
//     }
//     fsTraining.release();

//     // Load vocabulary
//     cv::FileStorage fsVocabulary;
//     fsVocabulary.open(vocabPath, cv::FileStorage::READ);
//     cv::Mat vocabulary;
//     fsVocabulary["Vocabulary"] >> vocabulary;
//     if (vocabulary.empty()) {
//         std::cerr << vocabPath << ": Vocabulary not found" << std::endl;
//         return;
//     }
//     fsVocabulary.release();

//     //load a chow-liu tree
//     cv::FileStorage fsTree;
//     fsTree.open(chowliutreePath, cv::FileStorage::READ);
//     cv::Mat clTree;
//     fsTree["ChowLiuTree"] >> clTree;
//     if (clTree.empty()) {
//         std::cerr << chowliutreePath << ": Chow-Liu tree not found" << std::endl;
//         return;
//     }
//     fsTree.release();

//     // Generate openFabMap object (FabMap2 - needs training bow data!)
//     int options = 0;
//     options |= cv::of2::FabMap::SAMPLED;
//     options |= cv::of2::FabMap::CHOW_LIU;
//     fabMap = new cv::of2::FabMap2(clTree, 0.39, 0, cv::of2::FabMap::SAMPLED |
//         cv::of2::FabMap::CHOW_LIU);
//     //add the training data for use with the sampling method
//     fabMap->addTraining(fabmapTrainData);

//     //  // Generate openFabMap object (FabMap1 with look up table)
//     //  int options = 0;
//     //  options |= of2::FabMap::MEAN_FIELD;
//     //  options |= of2::FabMap::CHOW_LIU;
//     //  //fabMap = new of2::FabMap(clTree, 0.39, 0, options);
//     //  fabMap = new of2::FabMapLUT(clTree, 0.39, 0, options, 3000, 6);
//     //  //fabMap = new of2::FabMapFBO(clTree, 0.39, 0, options, 3000, 1e-6, 1e-6, 512, 9);

//     // Create detector & extractor
//     detector = new cv::StarFeatureDetector(16, 10, 18, 18, 20);
//     // detector = new cv::DynamicAdaptedFeatureDetector(cv::AdjusterAdapter::create("STAR"), 130, 200, 5);
//     // detector = new DynamicAdaptedFeatureDetector (150, 200, 10,new FastAdjuster(20,true));

//     cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SURF(1000, 5, 7, true, false); // new cv::SIFT();

//     //use a FLANN matcher to generate bag-of-words representations
//     cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce" FlannBased
//     bide = new cv::BOWImgDescriptorExtractor(extractor, matcher);
//     bide->setVocabulary(vocabulary);

//     printConfusionMatrix = false;
//     confusionMat = cv::Mat(0, 0, CV_32F);

//     nextImageID = 0;
//     storage_retrival_counter_ = 0;
//     min_fabmap_baseline_ = 100;
//     valid = true;
// }

// FabMap::~FabMap()
// {
//     if (printConfusionMatrix)
//     {
//         std::ofstream writer("fabMapResult.txt");
//         for(int i = 0; i < confusionMat.rows; i++) {
//             for(int j = 0; j < confusionMat.cols; j++) {
//                 writer << confusionMat.at<float>(i, j) << " ";
//             }
//             writer << std::endl;
//         }
//         writer.close();
//     }
// }

// void FabMap::compareAndAdd(DataSpot3D::DataSpot3DPtr data_spot_ptr, int& out_newID, int& out_loopID)
// {

//     compareAndAdd(data_spot_ptr->getImageColor(), out_newID, out_loopID, data_spot_ptr->getKeyPoints());

//     if( out_newID >= 0 )
//     {
//         data_spot_ptr->setId(out_newID);
//     }

// }

// void FabMap::compareAndAdd(const cv::Mat& keyFrameImage, int& out_newID, int& out_loopID, std::vector<cv::KeyPoint>& out_kpts)
// {
//     // Convert keyframe image data to 3-channel OpenCV Mat (theoretically unneccessary)
//     cv::Mat frame;

//     keyFrameImage.convertTo(frame, CV_8UC1);
//     //cv::cvtColor(frame, frame, CV_GRAY2RGB);

//     // Generate FabMap bag-of-words data (image descriptor)
//     cv::Mat bow, bow2;
//     out_kpts.clear();
//     detector->detect(frame, out_kpts);
//     // std::cout << out_kpts.size() << "                      features detected " << std::endl;
//     // std::cout << "reached here!@" << std::endl;
//     if (out_kpts.empty())
//     {
//         out_newID = -1;
//         out_loopID = -1;
//         return;
//     }
//     bide->compute(frame, out_kpts, bow);

//     // Run FabMap
//     std::vector<cv::of2::IMatch> matches;
//     if (nextImageID > 0)
//         fabMap->compare(bow, matches);

//     bow_storage_.push_back(bow);
//     if (nextImageID > min_fabmap_baseline_)
//     {
//         fabMap->add(bow_storage_[storage_retrival_counter_++]);
//         // std::cout << bow_storage_[storage_retrival_counter_-1]<< std::endl << fabMap->getTestImgDescriptors()[storage_retrival_counter_-1] << std::endl;
//     }
//     // std::cout << bow.size() << std::endl;
//     std::cout << "fabmap test images size " << fabMap->getTestImgDescriptors().size() << std::endl;
//     // bow2 = bow;
//     out_newID = nextImageID;
//     ++nextImageID;

//     if (printConfusionMatrix)
//     {
//         cv::Mat resizedMat(nextImageID, nextImageID, confusionMat.type(), cv::Scalar(0));
//         if (confusionMat.rows > 0)
//             confusionMat.copyTo(resizedMat(cv::Rect(cv::Point(0, 0), confusionMat.size())));
//         confusionMat = resizedMat;

//         for(std::vector<cv::of2::IMatch>::iterator l = matches.begin(); l != matches.end(); ++ l)
//         {
//             int col = (l->imgIdx < 0) ? (nextImageID-1) : l->imgIdx;
//             confusionMat.at<float>(nextImageID-1, col) = l->match;
//         }
//     }

//     const float minLoopProbability = 0.55f*256;
//     float accumulatedProbability = 0;
//     const bool debugProbabilites = false;
//     if (debugProbabilites)
//         printf("FabMap probabilities:");

//     if (nextImageID > min_fabmap_baseline_)
//     {
//         for(std::vector<cv::of2::IMatch>::iterator l = matches.begin(); l != matches.end(); ++ l)
//         {
//             if (debugProbabilites)
//                 printf(" (%i: %f)", l->imgIdx, l->match);

//             if(l->imgIdx < 0)
//             {
//                 // Probability for new place
//                 accumulatedProbability += l->match*256;
//             }
//             else
//             {
//                 // int imageid = l->imgIdx;
//                 // int queryid = l->queryIdx;
//                 // Probability for existing place
//                 if (l->match*256 >= minLoopProbability)// && (queryid - imageid)>50)
//                 {
//                     out_loopID = l->imgIdx;
//                     std::cout << out_loopID << "                          LOOP ID IN FABMAP: \nmatch " << l->match*256 << "  likelihood  " << l->likelihood << " \n\n\n\n\n" << std::endl;

//                     // cv::waitKey(0);
//                     if (debugProbabilites)
//                         printf("\n");
//                     return;
//                 }
//                 accumulatedProbability += l->match*256;
//             }

//             if (! debugProbabilites && accumulatedProbability > 256 - minLoopProbability)
//                 break; // not possible anymore to find a frame with high enough probability
//         }

//         //-------------------------
//         // int match_id, queryid;
//         // float best_match = 0, likelihood;
//         // for(std::vector<cv::of2::IMatch>::iterator l = matches.begin(); l != matches.end(); l++) 
//         // {
//         //     // std::cout << " matchid " << l->imgIdx  << std::endl;
//         //     if(l->imgIdx < 0) 
//         //     {
//         //         // result_small.at<char>(l->queryIdx, l->queryIdx) =
//         //         //     (char)(l->match*255);
//         //             std::cout << "new image " << std::endl;

//         //     } 
//         //     else 
//         //     {
//         //         // result_small.at<char>(l->queryIdx, l->imgIdx) =
//         //         //     (char)(l->match*255);
//         //             float temp_match = l->match;
//         //             if (temp_match > best_match)
//         //             {
//         //                 best_match = temp_match;
//         //                 match_id = l->imgIdx;
//         //                 queryid = l->queryIdx;
//         //                 likelihood = l->likelihood;
//         //                 // std::cout << best_match << " id " << match_id << std::endl;
//         //             }
//         //             // std::cout << " previously seen image "<< l->imgIdx << " " << l->match*255 <<std::endl;
//         //     }
//         // }
//         // std::cout << "best match to " << match_id << " by " << best_match << " likelihood " << likelihood <<  std::endl;
//     }
//     if (debugProbabilites)
//         printf("\n");

//     out_loopID = -1;
//     return;
// }

// bool FabMap::isValid() const
// {
//     return valid;
// }


// };


// ==========================================================

    FabMap::FabMap()
{
    valid = false;

    std::string fabmapTrainDataPath = "/home/saif/msc_workspace/slam_x_lib/training_data/StLuciaShortTraindata.yml";
    std::string vocabPath =  "/home/saif/msc_workspace/slam_x_lib/training_data/StLuciaShortVocabulary.yml";
    std::string chowliutreePath =  "/home/saif/msc_workspace/slam_x_lib/training_data/StLuciaShortTree.yml";

    // Load training data
    cv::FileStorage fsTraining;
    fsTraining.open(fabmapTrainDataPath, cv::FileStorage::READ);
    cv::Mat fabmapTrainData;
    fsTraining["BOWImageDescs"] >> fabmapTrainData;
    if (fabmapTrainData.empty()) {
        std::cerr << fabmapTrainDataPath << ": FabMap Training Data not found" << std::endl;
        return;
    }
    fsTraining.release();

    // Load vocabulary
    cv::FileStorage fsVocabulary;
    fsVocabulary.open(vocabPath, cv::FileStorage::READ);
    cv::Mat vocabulary;
    fsVocabulary["Vocabulary"] >> vocabulary;
    if (vocabulary.empty()) {
        std::cerr << vocabPath << ": Vocabulary not found" << std::endl;
        return;
    }
    fsVocabulary.release();

    //load a chow-liu tree
    cv::FileStorage fsTree;
    fsTree.open(chowliutreePath, cv::FileStorage::READ);
    cv::Mat clTree;
    fsTree["ChowLiuTree"] >> clTree;
    if (clTree.empty()) {
        std::cerr << chowliutreePath << ": Chow-Liu tree not found" << std::endl;
        return;
    }
    fsTree.release();

    // Generate openFabMap object (FabMap2 - needs training bow data!)
    int options = 0;
    options |= cv::of2::FabMap::SAMPLED;
    options |= cv::of2::FabMap::CHOW_LIU;
    fabMap = new cv::of2::FabMap2(clTree, 0.39, 0, options);
    //add the training data for use with the sampling method
    fabMap->addTraining(fabmapTrainData);

    //  // Generate openFabMap object (FabMap1 with look up table)
    //  int options = 0;
    //  options |= of2::FabMap::MEAN_FIELD;
    //  options |= of2::FabMap::CHOW_LIU;
    //  //fabMap = new of2::FabMap(clTree, 0.39, 0, options);
    //  fabMap = new of2::FabMapLUT(clTree, 0.39, 0, options, 3000, 6);
    //  //fabMap = new of2::FabMapFBO(clTree, 0.39, 0, options, 3000, 1e-6, 1e-6, 512, 9);

    // Create detector & extractor
    detector = new cv::StarFeatureDetector(32, 10, 18, 18, 20);
    cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SURF(1000, 4, 2, true, true); // new cv::SIFT();

    //use a FLANN matcher to generate bag-of-words representations
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce" FlannBased
    bide = new cv::BOWImgDescriptorExtractor(extractor, matcher);
    bide->setVocabulary(vocabulary);

    printConfusionMatrix = false;
    confusionMat = cv::Mat(0, 0, CV_32F);

    nextImageID = 0;
    valid = true;
}

FabMap::~FabMap()
{
    if (printConfusionMatrix)
    {
        std::ofstream writer("fabMapResult.txt");
        for(int i = 0; i < confusionMat.rows; i++) {
            for(int j = 0; j < confusionMat.cols; j++) {
                writer << confusionMat.at<float>(i, j) << " ";
            }
            writer << std::endl;
        }
        writer.close();
    }
}

void FabMap::compareAndAdd(DataSpot3D::DataSpot3DPtr data_spot_ptr, int& out_newID, int& out_loopID){

    compareAndAdd(data_spot_ptr->getImageColor(), out_newID, out_loopID, data_spot_ptr->getKeyPoints());

    if( out_newID >= 0 ){
        data_spot_ptr->setId(out_newID);
    }

}

void FabMap::compareAndAdd(const cv::Mat& keyFrameImage, int& out_newID, int& out_loopID, std::vector<cv::KeyPoint>& out_kpts)
{
    // Convert keyframe image data to 3-channel OpenCV Mat (theoretically unneccessary)
    cv::Mat frame;

    keyFrameImage.convertTo(frame, CV_8UC1);
    //cv::cvtColor(frame, frame, CV_GRAY2RGB);

    // Generate FabMap bag-of-words data (image descriptor)
    cv::Mat bow;
    out_kpts.clear();
    detector->detect(frame, out_kpts);
    std::cout << "reached here!@" << std::endl;
    if (out_kpts.empty())
    {
        out_newID = -1;
        out_loopID = -1;
        return;
    }
    bide->compute(frame, out_kpts, bow);

    // Run FabMap
    std::vector<cv::of2::IMatch> matches;
    if (nextImageID > 0)
        fabMap->compare(bow, matches);
    fabMap->add(bow);
    out_newID = nextImageID;
    ++nextImageID;

    if (printConfusionMatrix)
    {
        cv::Mat resizedMat(nextImageID, nextImageID, confusionMat.type(), cv::Scalar(0));
        if (confusionMat.rows > 0)
            confusionMat.copyTo(resizedMat(cv::Rect(cv::Point(0, 0), confusionMat.size())));
        confusionMat = resizedMat;

        for(std::vector<cv::of2::IMatch>::iterator l = matches.begin(); l != matches.end(); ++ l)
        {
            int col = (l->imgIdx < 0) ? (nextImageID-1) : l->imgIdx;
            confusionMat.at<float>(nextImageID-1, col) = l->match;
        }
    }

    const float minLoopProbability = 0.55f;
    float accumulatedProbability = 0;
    const bool debugProbabilites = false;
    if (debugProbabilites)
        printf("FabMap probabilities:");
    for(std::vector<cv::of2::IMatch>::iterator l = matches.begin(); l != matches.end(); ++ l)
    {
        if (debugProbabilites)
            printf(" (%i: %f)", l->imgIdx, l->match);

        if(l->imgIdx < 0)
        {
            // Probability for new place
            accumulatedProbability += l->match;
        }
        else
        {
            // Probability for existing place
            if (l->match >= minLoopProbability)
            {
                out_loopID = l->imgIdx;
                if (debugProbabilites)
                    printf("\n");
                return;
            }
            accumulatedProbability += l->match;
        }

        if (! debugProbabilites && accumulatedProbability > 1 - minLoopProbability)
            break; // not possible anymore to find a frame with high enough probability
    }
    if (debugProbabilites)
        printf("\n");

    out_loopID = -1;
    return;
}

bool FabMap::isValid() const
{
    return valid;
}


};





