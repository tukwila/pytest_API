/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SlamPayloadAcceptor.h
 * @brief  ISlamAcceptor interface implementation class for slam output
 *******************************************************************************
 */
#ifndef SLAM_PAYLOAD_ACCEPTOR
#define SLAM_PAYLOAD_ACCEPTOR

#include <vector>
#include <opencv2/imgproc.hpp>
#include <string>
#include <sstream>
#include <memory>

#include "utilityFuns.h"
#include "algoInterface/ISlamAcceptor.h"
#include "CommunicateDef/RdbV2SSlam.h"

namespace roadDBCore
{


class SlamPayloadAcceptor : public algo::ISlamAcceptor
{
public:
    SlamPayloadAcceptor();
    virtual ~SlamPayloadAcceptor();

    /**
     *******************************************************************************
     * @brief get - get slam data
     *
     *  <1> Parameter Description:
     *
     *  @return  - SlamSnippetPayload_t &;
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual std::shared_ptr<SlamSnippetPayload_t> get();

    /**
     *******************************************************************************
     * @brief get - get slam data
     *
     *  <1> Parameter Description:
     *
     *  @return  - const SlamSnippetPayload_t &;
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual const std::shared_ptr<SlamSnippetPayload_t> get() const;

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
     * @brief putSnippet - add a Slam Snippet into slam payload
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
     * @brief putSnippets - add Slam Snippets into slam payload
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
     * @brief putNormCameraType - set eNormCameraType parameters
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
    virtual void setNormCameraType(NORM_CAMERA_TYPE_E eNormCameraType);  //can only one time

    /**
     *******************************************************************************
     * @brief setDescriptorType - set Descriptor Type parameters
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eDescriptorType
     *                 SLAM_DESCRIPTOR_TYPE_E,
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
     * @brief setScaleFactor - set Scale Factor parameters
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - eScaleFactor
     *                 SCALE_E,
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
     * @brief getKFs - get key Frame vector.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - void
     *
     *  @return key frame vector
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */

    bool getRefGps(Point3d_t &refGps);

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
    void putRefGps(Point3d_t &refGps);  //can only one time

    void clear();

private:
    Point3d_t refGps_;
    std::shared_ptr<SlamSnippetPayload_t> pSlamData_;

};

}

#endif
