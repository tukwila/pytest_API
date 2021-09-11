/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LocRTMatrixVecImp.h
 * @brief  implementation of interface of RT matrix vector
 *******************************************************************************
 */
#ifndef LOC_RTMATRIX_VEC_IMP_H
#define LOC_RTMATRIX_VEC_IMP_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "LocRTMatrixImp.h"
#include "algoInterface/ILocRTMatrixVec.h"

namespace roadDBCore
{

struct LocRTMatrixVecConfidence
{
    double len;
    double avgMatchedPointNum;

    LocRTMatrixVecConfidence(): len(0.0f),avgMatchedPointNum(0.0f) {}
};

/**
 *******************************************************************************
 * @class LocRTMatrixVecImp
 * @brief implementation of interface of RT matrix vector
 *
 *******************************************************************************
 */
class LocRTMatrixVecImp : public algo::ILocRTMatrixVec
{
public:
    LocRTMatrixVecImp() {}
    virtual ~LocRTMatrixVecImp() {}

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
    bool putStart();

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
    bool putEnd();

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
    bool put(IN uint64_t sectionID,
             IN uint64_t referenceID,
             IN uint64_t version,
             IN uint32_t frmIdx,
             IN bool isKeyFrame,
             IN cv::Mat &rtMat);

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
    bool putConfidence(IN double len, IN double agvMatchPoint);

   /**
     *******************************************************************************
     * @brief get - get all datas
     *
     *  <1> Parameter Description:
     *
     *  @param [Out]  - std::vector<LocRTMatrixImp *> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    void get(std::vector<std::shared_ptr<LocRTMatrixImp>> &vecDatas);

   /**
     *******************************************************************************
     * @brief get - get RT confidence of one connected domain
     *
     *  <1> Parameter Description:
     *
     *  @param [Out]  - double& confidence,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    bool getConfidence(OUT LocRTMatrixVecConfidence& confidence);
private:
    //LocRTMatrixVecImp(const LocRTMatrixVecImp &) = delete;
    //LocRTMatrixVecImp &operator=(const LocRTMatrixVecImp &) = delete;
    LocRTMatrixVecConfidence rtConfidence_;

public:
    std::shared_ptr<LocRTMatrixImp> spLocRTMatrixImp_;
    std::vector<std::shared_ptr<LocRTMatrixImp>> vecDatas_;

};

}
#endif
