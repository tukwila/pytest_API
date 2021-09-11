/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IRTMatrixVec.h
 * @brief  interface of RT matrix vector
 *******************************************************************************
 */
#ifndef _IRTMATRIX_VEC_H
#define _IRTMATRIX_VEC_H

#include <opencv2/imgproc.hpp>
#include "typeDef.h"
#include "IRTMatrix.h"

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
class IRTMatrixVec
{
public:
    /**
      *******************************************************************************
      * @brief putStart - called at the start of update one segment, and while alloc memory for datas
      *
      *  <1> Parameter Description:
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
    */
    virtual bool putStart() = 0;

    /**
      *******************************************************************************
      * @brief putEnd - called at the end of update one segment
      *
      *  <1> Parameter Description:
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
    */
    virtual bool putEnd() = 0;

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
    virtual bool put(IN int32_t frmIdx, IN const cv::Mat &mat, IN bool isKeyFrame) = 0;
};
}
#endif


