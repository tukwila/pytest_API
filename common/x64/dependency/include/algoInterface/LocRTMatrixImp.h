/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LocRTMatrixImp.h
 * @brief  implementation of Localization RT matrix
 *******************************************************************************
 */
#ifndef LOC_RTMATRIX_IMP_H
#define LOC_RTMATRIX_IMP_H

#include <map>
#include <opencv2/opencv.hpp>
#include <boost/serialization/base_object.hpp>
#include "serialization/rdb/RdbSeriaCommon.h"
#include "algoInterface/ILocRTMatrix.h"
#include "CommunicateDef/RdbV2SCommon.h"

namespace roadDBCore
{

struct RTMatrixItem_t
{
    uint32_t kfIdx;
    bool     isKeyFrame;
    cv::Mat  rtMat;

    RTMatrixItem_t(): kfIdx (0), isKeyFrame(true) {}
    RTMatrixItem_t(uint32_t kfIdxIn, bool isKeyFrameIn, cv::Mat &rtMatIn):
                         kfIdx (kfIdxIn), isKeyFrame(isKeyFrameIn), rtMat(rtMatIn)
                         {}
};

struct RTItemsInSection_t
{
    uint64_t sectionID;

    uint64_t referenceID;

    /* Section version or reference version in the section. */
    uint64_t version;

    std::vector<RTMatrixItem_t> vecRTItems;

    RTItemsInSection_t(): sectionID (0LL), referenceID(0LL), version(0LL) {}
    RTItemsInSection_t(uint64_t sectionIDIn, uint64_t refferenceIDIn,
                       uint64_t versionIn,
                       uint32_t kfIdxIn, bool isKeyFrameIn,
                       cv::Mat &rtMatIn):
                       sectionID (sectionIDIn),
                       referenceID(refferenceIDIn),
                       version(versionIn)
    {
        vecRTItems.emplace_back(kfIdxIn, isKeyFrameIn, rtMatIn);
    }
};

struct RTMatrixPayload_t: public PayloadBase_t
{
    std::vector<RTItemsInSection_t> rtSections;
    bool loadRTSectionsByTxt(std::string RtfileName, RoadDatabaseHead_t &header);
    RTMatrixPayload_t(SNIPPET_PAYLOAD_TYPE_E  payloadType = SNIPPET_PAYLOAD_TYPE_RT_E):
                           PayloadBase_t(payloadType)
    {
    }
};

struct RTMatrixTxtItem_t
{
    uint64_t sectionID;
    uint64_t referenceID;
    uint64_t version;
    uint32_t kfIdx;
    bool     isKeyFrame;
    cv::Mat  rtMat;
    
    RTMatrixTxtItem_t(uint64_t sectionIDIn, uint64_t referenceIDIn,uint64_t versionIn, uint32_t kfIdxIn, bool isKeyFrameIn, cv::Mat &rtMatIn):
                    sectionID (sectionIDIn),referenceID(referenceIDIn),version(versionIn),kfIdx (kfIdxIn), isKeyFrame(isKeyFrameIn), rtMat(rtMatIn)
    {}
    RTMatrixTxtItem_t(): sectionID(0), referenceID(0), version(0), kfIdx(0), isKeyFrame(false), rtMat()
    {
    }
};

struct RTMatrixTxtPayload_t: public PayloadBase_t
{
    std::vector<RTMatrixTxtItem_t> rtTxtItems;
    bool initialize(std::string RtfileName);
};

/**
 *******************************************************************************
 * @class LocRTMatrixImp
 * @brief
 *
 *  get the RT matrix of each image from the interface
 *******************************************************************************
 */
class LocRTMatrixImp : public algo::ILocRTMatrix
{
public:
     LocRTMatrixImp() {}
     virtual ~LocRTMatrixImp() {}
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
    bool get(IN uint32_t frmIdx,
             OUT uint64_t &sectionID,
             OUT uint64_t &referenceID,
             OUT uint64_t &version,
             OUT bool &isKeyFrame,
             OUT cv::Mat &rtMat);
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
    bool getStartFrameIndex(uint32_t &index);

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
    bool getEndFrameIndex(uint32_t &index);

    const std::vector<RTItemsInSection_t> & get() const
    {
        return vecRTMatrixInfo_;
    }

    std::vector<RTItemsInSection_t> & get()
    {
        return vecRTMatrixInfo_;
    }

    void set(const std::vector<RTItemsInSection_t> &vecRTMatrixInfo)
    {
        vecRTMatrixInfo_ = vecRTMatrixInfo;
    }

private:
    std::vector<RTItemsInSection_t> vecRTMatrixInfo_;
};

}
#endif
