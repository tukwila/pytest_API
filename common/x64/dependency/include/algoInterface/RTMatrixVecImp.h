/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RTMatrixImpl.h
 * @brief  implementation of interface of RT matrix vector
 *******************************************************************************
 */
#ifndef _IRTMATRIX_VEC_IMP_H
#define _IRTMATRIX_VEC_IMP_H

#include "IRTMatrixVec.h"
#include "RTMatrixImp.h"
#include <opencv2/opencv.hpp>

namespace roadDBCore
{


/**
 *******************************************************************************
 * @class RTMatrixVecImp RTMatrixVecImp.h
 * @brief implementation of interface of RT matrix vector
 *
 *******************************************************************************
 */
class RTMatrixVecImp :public algo::IRTMatrixVec
{
public:
    RTMatrixVecImp();
    virtual ~RTMatrixVecImp();

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
    virtual bool putStart();

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
    virtual bool putEnd();

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
    virtual bool put(IN int32_t frmIdx, IN const cv::Mat &mat, IN bool isKeyFrame);

    /**
     *******************************************************************************
     * @brief get - get all datas
     *
     *  <1> Parameter Description:
     *
     *  @param [Out]  - std::vector<RTMatrixImp *> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    void get(std::vector<RTMatrixImp *> &vecDatas);

private:
    RTMatrixVecImp(const RTMatrixVecImp &) = delete;
    RTMatrixVecImp &operator=(const RTMatrixVecImp &) = delete;

public:
    RTMatrixImp * pImp_;
    std::vector<RTMatrixImp *> vecDatas_;

};

}
#endif
