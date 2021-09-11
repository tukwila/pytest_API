/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IRTMatrix.h
 * @brief  interface of RT matrix
 *******************************************************************************
 */
#ifndef IRTMATRIX_H
#define IRTMATRIX_H

#include <opencv2/imgproc.hpp>
#include "typeDef.h"

namespace algo
{
/**
 *******************************************************************************
 * @class IRTMatrixIn IRTMatrix.h
 * @brief interface of IRTMatrixIn
 *
 *  get the RT matrix of each image from the interface
 *******************************************************************************
 */
  using roadDBCore::int32_t;
class IRTMatrixIn
{
public:
    virtual ~IRTMatrixIn() {};
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
      *  @param [Out]  - isKeyFrame
      *                  true: is key frame, false: not key frame
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool get(IN int32_t frmIdx, OUT cv::Mat &mat, OUT bool &isKeyFrame) = 0;

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
    virtual int32_t getStartFrameIndex() = 0;

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
    virtual int32_t getEndFrameIndex() = 0;
};

/**
 *******************************************************************************
 * @class IRTMatrixOut IRTMatrix.h
 * @brief interface of IRTMatrixOut
 *
 *  put the RT matrix of each image from the interface
 *******************************************************************************
 */
class IRTMatrixOut
{
public:
    virtual ~IRTMatrixOut() {};
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
      *  @param [In]  - isKeyFrame
      *                 true: is key frame, false: not key frame
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool put(IN int32_t frmIdx, IN const cv::Mat &mat, IN bool isKeyFrame) = 0;
};
}
#endif
