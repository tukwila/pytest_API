/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   KeyFrameDef.h
 * @brief  basic structure of key frame
 *******************************************************************************
 */

#ifndef KEYFRAME_DEF_H_
#define KEYFRAME_DEF_H_

#include <vector>
#include <opencv2/imgproc.hpp>
#include "CommunicateDef/Communicate.h"

namespace algo {

struct KeyFrameIdx
{
    std::size_t  idx_; // KeyFrame Index
    cv::KeyPoint kpt_; // 2D KeyPoint data

    cv::Mat      desc_; // Keypoint descriptor

    KeyFrameIdx() : idx_(0), kpt_(), desc_() {}
};

struct MapPoint3D
{
    std::vector<KeyFrameIdx> vkfIdx_; // vector of KeyFrames which observed this 3D point
    cv::Mat pos_;                   // 3D world coordinates of this MapPoint
    cv::Mat descriptor_;            // feature descriptor which is the combination of
                                   // all observers information

    MapPoint3D() : pos_(), descriptor_()
    {
        vkfIdx_.reserve(5);   ///adjust this reserve capacity size to get the balance of performance and space
    }
};

struct SlamData
{
    IntrinsicData             camParam_; // Camera intrinsic parameters
    cv::Point3d               refGPS_;   // Reference GPS
    std::vector<MapPoint3D>   vMP3D_;    // All 3D points output of SLAM
    std::vector<KeyFrameData> vKFD_;     // All KeyFrames data output of SLAM

    SlamData() : camParam_(), refGPS_()
    {
        vMP3D_.reserve(1024);  ///adjust this reserve capacity size to get the balance of performance and space
        vKFD_.reserve(128);    ///adjust this reserve capacity size to get the balance of performance and space
    }
};

}


#endif
