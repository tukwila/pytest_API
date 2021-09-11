/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2017-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SSeriaImpCommon.h
 * @brief  Implementation of data serialization and deserialization
 *         for common data types
 *******************************************************************************
 */

#include <opencv2/imgproc.hpp>
#include <utility>
#include <memory>
#include "CommunicateDef/RdbV2SCommon.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"
#include "algoInterface/RTMatrixImp.h"
#include "algoInterface/LocRTMatrixImp.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "CommunicateDef/RdbV2SSlam.h"

#ifndef RDB_V2S_SERIA_IMP_COMMON_H
#define RDB_V2S_SERIA_IMP_COMMON_H

namespace roadDBCore
{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RoadDatabaseHead_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, RoadDatabaseHead_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, RoadDatabaseHead_t &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize bool data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, bool &obj)
{
    uint8_t ch = static_cast<uint8_t>(obj);

    seria & ch;
    obj = static_cast<uint8_t>(ch);
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize cv::Size data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, cv::Size &obj)
{
    seria & obj.height & obj.width;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize cv::Mat data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, cv::Mat &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, cv::Mat &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixImp data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, RTMatrixImp &obj)
{
    seria & obj.get();
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixImp::RTMatrixTag data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, RTMatrixImp::RTMatrixTag &obj)
{
    seria & obj.rtMatrix & obj.isKeyFrame;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LocRTMatrixImp data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, LocRTMatrixImp &obj)
{
    seria & obj.get();
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixPayload_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */

template<>
void serialize(RdbV2SBinSerializer &seria, RTMatrixPayload_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, RTMatrixPayload_t &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixTxtPayload_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, RTMatrixTxtPayload_t &obj)
{
    seria & obj.rtTxtItems;
}


/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixTxtItem_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, RTMatrixTxtItem_t &obj)
{
    seria & obj.sectionID & obj.referenceID & obj.version & obj.kfIdx & obj.isKeyFrame & obj.rtMat;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTItemsInSection_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, RTItemsInSection_t &obj)
{
    seria & obj.sectionID & obj.referenceID & obj.version & obj.vecRTItems;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize RTMatrixItem_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, RTMatrixItem_t &obj)
{
    seria & obj.kfIdx & obj.isKeyFrame & obj.rtMat;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize T (&array)[DIM] data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria, typename T, int DIM>
void serialize(Seria &seria, T (&array)[DIM])
{
    for(auto &element: array)
    {
        seria & element;
    }

    return;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::pair<T1, T2> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria, typename T1, typename T2>
void serialize(Seria &seria, std::pair<T1, T2> &obj)
{
    seria & obj.first & obj.second;

    return;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<T> data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template <class T/*, class D, class Alloc*/>
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<T/*, D, Alloc*/> &sharedPointer)
{
    bool bValid = static_cast<bool>(sharedPointer);

    seria & bValid;

    if (bValid)
    {
        seria & *sharedPointer;
    }
    else
    {
        COM_LOG_WARN << "NULL shared pointer for type: "
                     << typeid(*sharedPointer).name();
    }
}

template <class T/*, class D, class Alloc*/>
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<T/*, D, Alloc*/> &sharedPointer)
{
    bool bValid = false;

    seria & bValid;

    if (bValid)
    {
        sharedPointer = std::make_shared<T/*, D, Alloc*/>();
        seria & *sharedPointer;
    }
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ObserveKfInfo_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, ObserveKfInfo_t &obj)
{
    seria & obj.startKfIdx & obj.endKfIdx;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LANE_MARKING_TYPE_E data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, LANE_MARKING_TYPE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LANE_MARKING_TYPE_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SCALE_E data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, SCALE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, SCALE_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SLAM_DESCRIPTOR_TYPE_E data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, SLAM_DESCRIPTOR_TYPE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, SLAM_DESCRIPTOR_TYPE_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DB_POINT_TYPE_E data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, DB_POINT_TYPE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, DB_POINT_TYPE_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ModelConfigBase_t data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<typename Seria>
void serialize(Seria &seria, ModelConfigBase_t &obj)
{
    seria & obj.fx
          & obj.fy
          & obj.cx
          & obj.cy
          & obj.yaw
          & obj.pitch
          & obj.roll
          & obj.cameraHeight
          & obj.length
          & obj.width
          & obj.slamScale
          & obj.descriptorType
          & obj.basisMat
          & obj.baseTimestamp;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LANE_MARKING_COLOR_TYPE_E data.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] seria    RdbV2SBinSerializer or RdbV2SBinDeserializer object
 *
 *  @param [In/Out] obj  Serialized or Deserialized data.
 *
 *  @return void
 *******************************************************************************
 */
template<>
void serialize(RdbV2SBinSerializer &seria, LANE_MARKING_COLOR_TYPE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LANE_MARKING_COLOR_TYPE_E &obj);

}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_SERIA_IMP_COMMON_H



