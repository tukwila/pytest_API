/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RTMatrixImpl.h
 * @brief  implementation of interface of RT matrix
 *******************************************************************************
 */
#ifndef IRTMATRIX_IMP_H
#define IRTMATRIX_IMP_H

#include <map>
#include <opencv2/opencv.hpp>
#include <boost/serialization/base_object.hpp>
#include "algoInterface/IRTMatrix.h"
#include "serialization/rdb/RdbSeriaCommon.h"

namespace roadDBCore
{


/**
 *******************************************************************************
 * @class IRTMatrixIn IRTMatrix.h
 * @brief interface of IRTMatrixIn
 *
 *  get the RT matrix of each image from the interface
 *******************************************************************************
 */
class RTMatrixImp :public algo::IRTMatrixOut, public algo::IRTMatrixIn
{
public:
     RTMatrixImp();
     ~RTMatrixImp();
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
    virtual bool get(IN roadDBCore::int32_t frmIdx, OUT cv::Mat &mat,OUT bool &isKeyFrame);
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
    virtual bool put(IN int32_t frmIdx, IN const cv::Mat &mat,IN bool isKeyFrame);

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
    virtual int32_t getStartFrameIndex();

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
    virtual int32_t getEndFrameIndex();

    struct RTMatrixTag
    {
       cv::Mat  rtMatrix;
       bool     isKeyFrame;
    };

    const std::map<int32_t,RTMatrixTag>& get() const
    {
        return rtMatrix_;
    }
    
    std::map<int32_t,RTMatrixTag>& get()
    {
        return rtMatrix_;
    }

    void set(const std::map<int32_t,RTMatrixTag>& matrix_map)
    {
        rtMatrix_ = matrix_map;
    }

    void debugString(std::stringstream &ss, bool bDetail=true);

private:
    typedef std::map<int32_t, RTMatrixTag> MapMatrix;
    MapMatrix  rtMatrix_;
};

}
#endif
