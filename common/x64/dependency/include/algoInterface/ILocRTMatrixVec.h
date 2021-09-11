/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ILocRTMatrixVec.h
 * @brief  interface of RT matrix vector
 *******************************************************************************
 */
#ifndef I_LOC_RTMATRIX_VEC_H
#define I_LOC_RTMATRIX_VEC_H

#include <memory>
#include "ILocRTMatrix.h"
#include <opencv2/opencv.hpp>

namespace algo
{


/**
 *******************************************************************************
 * @class ILocRTMatrixVec
 * @brief  interface of RT matrix vector
 *
 *******************************************************************************
 */
class ILocRTMatrixVec
{
public:
    virtual ~ILocRTMatrixVec() {};

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
    virtual bool put(IN uint64_t sectionID,
                     IN uint64_t referenceID,
                     IN uint64_t version,
                     IN uint32_t frmIdx,
                     IN bool isKeyFrame,
                     IN cv::Mat &rtMat) = 0;

    /**
      *******************************************************************************
      * @brief put - put RT confidence of one connected domain
      *
      *  <1> Parameter Description:
      *
      *  @param [In]  - len   RT total len
      *
      *  @param [In]  - agvMatchPoint   average matched point in vehicleDB
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool putConfidence(IN double len, IN double agvMatchPoint) = 0;
};

}
#endif

