/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ISlamCollector.h
 * @brief  interface of slam output collector
 *******************************************************************************
 */

#ifndef ISLAM_COLLECTOR_H
#define ISLAM_COLLECTOR_H

#include "typeDef.h"
#include "CommunicateDef/RdbV2SSlam.h"

namespace algo {

using roadDBCore::int32_t;

/**
 *******************************************************************************
 * @class ISlamCollector ISlamCollector.h
 * @brief interface of slam output collector
 *
 *
 *******************************************************************************
 */
class ISlamCollector
{
public:

    virtual ~ISlamCollector(){};
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
     * @brief putKF - add one key frame
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - kf
     *                 roadDBCore::KeyFrameSR_t &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putKF(uint32_t snippetID, roadDBCore::KeyFrameSR_t &kf) = 0;

    /**
     *******************************************************************************
     * @brief putMapPoint - add one 3d map point
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - mp
     *                 roadDBCore::Inc3DLmPointSR_t &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putMP(uint32_t snippetID, roadDBCore::Inc3DLmPointSR_t &mp) = 0;

    /**
     *******************************************************************************
     * @brief putKFs - add all key frame data
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - kfs
     *                 std::vector<roadDBCore::KeyFrameSR_t> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putKFs(uint32_t snippetID, std::vector<roadDBCore::KeyFrameSR_t> &kfs) = 0;

    /**
     *******************************************************************************
     * @brief putMapPoints - add all 3d map points
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - mps
     *                 std::vector<roadDBCore::Inc3DLmPointSR_t> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putMPs(uint32_t snippetID, std::vector<roadDBCore::Inc3DLmPointSR_t> &mps) = 0;

    /**
     *******************************************************************************
     * @brief putRefGps - set reference GPS
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - refGps
     *                 Point3d_t &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putRefGps(roadDBCore::Point3d_t &refGps) = 0;  //can only one time


    /**
     *******************************************************************************
     * @brief putSnippet - Add slam snippet into slam payload
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - snippet
     *                 roadDBCore::SlamSnippetSR_t &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putSnippet(const std::shared_ptr<roadDBCore::SlamSnippetSR_t> &spSnippet) = 0;

    /**
     *******************************************************************************
     * @brief putSnippets - Add slam snippets into slam payload
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vecSnippets
     *                 std::vector<roadDBCore::SlamSnippetSR_t> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putSnippets(const std::vector<std::shared_ptr<roadDBCore::SlamSnippetSR_t>> &vecSpSnippets) = 0;

    /**
     *******************************************************************************
     * @brief setDescriptorType - set Descriptor Type
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eDescriptorType
     *                 roadDBCore::SLAM_DESCRIPTOR_TYPE_E &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setDescriptorType(roadDBCore::SLAM_DESCRIPTOR_TYPE_E eDescriptorType) = 0;

    /**
     *******************************************************************************
     * @brief setNormCameraType - set eNormCameraType parameters
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eNormCameraType
     *                 roadDBCore::NORM_CAMERA_TYPE_E &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setNormCameraType(roadDBCore::NORM_CAMERA_TYPE_E eNormCameraType) = 0;

    /**
     *******************************************************************************
     * @brief setScaleFactor - set Scale Factor parameters
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eScaleFactor
     *                 roadDBCore::SCALE_E &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setScaleFactor(roadDBCore::SCALE_E eScaleFactor) = 0;

    /**
     *******************************************************************************
     * @brief get - get slam payload handle, in order to fill data into it
     *
     *  <1> Parameter Description:
     *
     *  @return status - Handle of slam payload
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual roadDBCore::SlamSnippetPayload_t * getSlamPldHandle() = 0;

};

}


#endif

