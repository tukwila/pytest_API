/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SlamPayloadsCollector.h
 * @brief  implementation of interface of ISlamCollector for slam output
 *******************************************************************************
 */

#ifndef SLAM_PAYLOADS_COLLECTOR
#define SLAM_PAYLOADS_COLLECTOR

#include "typeDef.h"
#include "ISlamCollector.h"
#include "SlamPayloadAcceptor.h"
#include <iostream>

namespace roadDBCore
{


class SlamPayloadsCollector : public algo::ISlamCollector
{
public:
    SlamPayloadsCollector();
    ~SlamPayloadsCollector();

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
     * @brief putKF - add one key frame
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - kf
     *                 KeyFrameSR_t &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putKF(uint32_t snippetID, KeyFrameSR_t &kf);

    /**
     *******************************************************************************
     * @brief putMapPoint - add one 3d map point
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - mp
     *                 Inc3DLmPointSR_t &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putMP(uint32_t snippetID, Inc3DLmPointSR_t &mp);

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
    virtual void putRefGps(Point3d_t &refGps);  //can only one time

    /**
     *******************************************************************************
     * @brief putKFs - add all key frame data
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - kfs
     *                 std::vector<KeyFrameSR_t> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putKFs(uint32_t snippetID, std::vector<KeyFrameSR_t> &kfs);

    /**
     *******************************************************************************
     * @brief putMapPoints - add all 3d map points
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - mps
     *                 std::vector<Inc3DLmPointSR_t> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putMPs(uint32_t snippetID, std::vector<Inc3DLmPointSR_t> &mps);

    /**
     *******************************************************************************
     * @brief putSnippet - Add slam snippet into slam payload
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spSnippet
     *                 Share pointer to a Slam Snippet
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putSnippet(const std::shared_ptr<SlamSnippetSR_t> &spSnippet);

    /**
     *******************************************************************************
     * @brief putSnippets - Add slam snippets into slam payload
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vecSpSnippets
     *                 slam snippets
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void putSnippets(const std::vector<std::shared_ptr<SlamSnippetSR_t>> &vecSpSnippets);

    /**
     *******************************************************************************
     * @brief setDescriptorType - set Descriptor Type
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eDescriptorType
     *                 SLAM_DESCRIPTOR_TYPE_E &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setDescriptorType(SLAM_DESCRIPTOR_TYPE_E eDescriptorType);

    /**
     *******************************************************************************
     * @brief setNormCameraType - set eNormCameraType parameters
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eNormCameraType
     *                 NORM_CAMERA_TYPE_E &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setNormCameraType(NORM_CAMERA_TYPE_E eNormCameraType);

    /**
     *******************************************************************************
     * @brief setScaleFactor - set Scale Factor parameters
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eScaleFactor
     *                 SCALE_E &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setScaleFactor(SCALE_E eScaleFactor);

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
    virtual SlamSnippetPayload_t * getSlamPldHandle();

    /**
     *******************************************************************************
     * @brief get - get all datas
     *
     *  <1> Parameter Description:
     *
     *  @param [Out]  - std::vector<SlamPayloadAcceptor *> &,
     *
     *  @return status -
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    void getPayloadAcceptor(std::vector<std::shared_ptr<SlamPayloadAcceptor> > &vecSpPayloadAcceptors);

private:
    SlamPayloadsCollector(const SlamPayloadsCollector&) = delete;
    SlamPayloadsCollector& operator=(const SlamPayloadsCollector&) = delete;

    std::shared_ptr<SlamPayloadAcceptor> spPayloadAcceptor_;
    std::vector<std::shared_ptr<SlamPayloadAcceptor> > vecSpPayloadAcceptors_;
};

}


#endif


