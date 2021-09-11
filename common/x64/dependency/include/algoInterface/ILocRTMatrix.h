/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ILocRTMatrix.h
 * @brief  interface of Localization RT matrix
 *******************************************************************************
 */
#ifndef I_LOC_RTMATRIX_H
#define I_LOC_RTMATRIX_H

#include <opencv2/opencv.hpp>
#include <boost/serialization/base_object.hpp>
#include "serialization/rdb/RdbSeriaCommon.h"

namespace algo
{

/**
 *******************************************************************************
 * @class ILocRTMatrix
 * @brief
 *
 *  get the RT matrix of each image from the interface
 *******************************************************************************
 */
class ILocRTMatrix
{
public:
     virtual ~ILocRTMatrix() {};
    /**
      *******************************************************************************
      * @brief get - get RT matrix of a image by index
      *
      *  <1> Parameter Description:
      *
      *  @param [In]  - frmIdx  index of each image in file
      *
      *  @param [Out]  - mat   the RT matrix of the image
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool get(IN uint32_t frmIdx,
                     OUT uint64_t &sectionID,
                     OUT uint64_t &referenceID,
                     OUT uint64_t &version,
                     OUT bool &isKeyFrame,
                     OUT cv::Mat &rtMat) = 0;
    /**
      *******************************************************************************
      * @brief put - put image and the corresponding index
      *
      *  <1> Parameter Description:
      *
      *  @param [In]  - frmIdx   index of each image in file
      *
      *  @param [In]  - mat   the image
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool put(IN uint64_t sectionID,
                     IN uint64_t referenceID,
                     IN uint64_t version,
                     IN uint32_t frmIdx,
                     IN bool isKeyFrame,
                     IN cv::Mat &rtMat) = 0;

    /**
      *******************************************************************************
      * @brief getStartFrameIndex - get the start frame index
      *
      *  <1> Parameter Description:
      *
      *  @return  - the start frame index
      *
      *  <2> Detailed Description:
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool getStartFrameIndex(uint32_t &index) = 0;

    /**
      *******************************************************************************
      * @brief getEndFrameIndex - get the end frame index
      *
      *  <1> Parameter Description:
      *
      *  @return  - the start frame index
      *
      *  <2> Detailed Description:
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool getEndFrameIndex(uint32_t &index) = 0;
};

}
#endif

