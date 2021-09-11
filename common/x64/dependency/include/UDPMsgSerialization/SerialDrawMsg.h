/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    SerialDrawMsg.h
 * @brief   Serial and Deserial DrawMessage.
 *******************************************************************************
 */
#ifndef SERIAL_DRAW_MSG__H__
#define SERIAL_DRAW_MSG__H__

#include "DrawMessage.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Werror"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#endif
#include "serialization/rdb/RdbBinSeriaBase.h"
#include "serialization/rdb/RdbBinDeseriaBase.h"
#include "serialization/rdb/RdbSeriaSize.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"
#include "serialization/rdb/RdbSeriaInterface.h"
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace roadDBCore
{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief rdbBinSStreamSeria - Serialize object to string
 *
 *  <1> Parameter Description:
 *
 *  @param [In] - object
 *              - uReserveLen
 *
 *  @param [Out] - str
 *
 *  @return ROAD_DATABASE_OK or other error code
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinSStreamSeria(const T &object, std::string &str, uint32_t uReserveLen)
{
    uint32_t status = ROAD_DATABASE_OK;
    std::stringstream sstream;
    str.resize(uReserveLen);
    
    if (!sstream.good())
    {
        COM_LOG_ERROR <<errorCode(RDB_SERIA_ERR_OPEN_FILE)<< "Failed to open string stream.";

        return RDB_SERIA_ERR_OPEN_FILE;
    }

    status = rdbBinStreamSeria(object, sstream);

    if (ROAD_DATABASE_OK == status)
    {
        str.insert(uReserveLen, sstream.str());
    }

    return status;
}

/**
 *******************************************************************************
 * @brief rdbBinSStreamDeseria - Deserialize object from string
 *
 *  <1> Parameter Description:
 *
 *  @param [In] - str
 *              - uReserveLen
 *
 *  @param [Out] - object
 *
 *  @return ROAD_DATABASE_OK or other error code
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinSStreamDeseria(T &object, const std::string &str, uint32_t uReserveLen)
{
    std::stringstream sstream;

    sstream.str(std::move(std::string(str.begin() + uReserveLen, str.end())));

    if (!sstream.good())
    {
        COM_LOG_ERROR <<errorCode(RDB_SERIA_ERR_OPEN_FILE)<<"Failed to open string stream.";

        return RDB_SERIA_ERR_OPEN_FILE;
    }

    return rdbBinStreamDeseria(object, sstream);
}

template<>
void serialize(RdbV2SBinDeserializer &seria,  std::shared_ptr<MessageObject_t> &obj);

template<>
void serialize(RdbV2SBinSerializer &seria,  std::shared_ptr<MessageObject_t> &obj);

template<typename Seria>
void serialize(Seria &seria, cv::Point3f &obj)
{
    seria & obj.x
          & obj.y
          & obj.z;
}

template<typename Seria>
void serialize(Seria &seria, cv::Point2f &obj)
{
    seria & obj.x
          & obj.y;
}

template<typename Seria>
void serialize(Seria &seria, cv::Point2d &obj)
{
    seria & obj.x
          & obj.y;
}

template<typename Seria>
void serialize(Seria &seria, algo_vector3t &obj)
{
    seria & obj[0]
          & obj[1]
          & obj[2];
}

template<>
void serialize(RdbV2SBinSerializer &seria, algo_matrix3t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, algo_matrix3t &obj);

template<>
void serialize(RdbV2SBinSerializer &seria, algo_matrixXt &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, algo_matrixXt &obj);

//seria predict
template<>
void serialize(RdbV2SBinSerializer &seria, MessagePredict_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, MessagePredict_t &obj);

template<>
void serialize(RdbV2SBinSerializer &seria, MessageUpdate_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, MessageUpdate_t &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageGps_t data.
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
void serialize(Seria &seria, MessageGps_t &obj)
{
    int selectionMode = obj.selectionMode;
    int fixMode = obj.fixMode;
    int positionStatus = obj.positionStatus;
    int magneticVariationDir = obj.magneticVariationDir;

    seria & obj.frameID
          & obj.gps
          & obj.speed
          & obj.timeStamp
          & obj.channleID
          & obj.usedSatNum
          & obj.trackAngle
          & obj.isV2Valid
          & obj.oriSpeed
          & obj.hdop
          & obj.pdop
          & obj.vdop
          & obj.magneticVariation
          & obj.gpsQualityIndicator
          & obj.ageOfDgps
          & obj.dgpsStationId
          & selectionMode
          & fixMode
          & positionStatus
          & magneticVariationDir;

    obj.selectionMode = (char)selectionMode;
    obj.fixMode = (char)fixMode;
    obj.positionStatus = (char)positionStatus;
    obj.magneticVariationDir = (char)magneticVariationDir;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageImg_t data.
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
void serialize(Seria &seria, MessageImg_t &obj)
{
    uint8_t flag = static_cast<uint8_t>(obj.imageUndistortion);

    seria & obj.frameID
          & obj.imgStatus
          & obj.relTimeStamp
          & obj.arrivalTimestamp
          & obj.channleID
          & obj.vecCharImage
          & flag;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageCan_t data.
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
void serialize(Seria &seria, MessageCan_t &obj)
{
    seria & obj.frameID
          & obj.arrivalTimestamp
          & obj.fValue
          & obj.sDataType;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageSensorInfo_t data.
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
void serialize(Seria &seria, MessageSensorInfo_t &obj)
{
    seria & obj.iChannleID
          & obj.sSensorType
          & obj.sInfoType
          & obj.sInfoValue;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ImuRecord_t data.
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
void serialize(Seria &seria, ImuRecord_t &obj)
{
    seria & obj.frameID
          & obj.value
          & obj.relTimeStamp
          & obj.arrivalTimestamp;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageImuPack_t data.
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
void serialize(Seria &seria, MessageImuPack_t &obj)
{
    seria & obj.imuAcc
          & obj.imuGyro
          & obj.imuOri;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageObdPack_t data.
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
void serialize(Seria &seria, MessageObdPack_t &obj)
{
    seria & obj.number
          & obj.timestamp
          & obj.speed;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageCamInParam_t data.
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
void serialize(Seria &seria, MessageCamInParam_t &obj)
{
    seria & obj.fx
          & obj.fy
          & obj.cx
          & obj.cy
          & obj.k1
          & obj.k2
          & obj.k3
          & obj.p1
          & obj.p2
          & obj.scaleFactor;
    
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessagePMBA_t data.
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
void serialize(Seria &seria, MessagePMBA_t &obj)
{
    seria & obj.frameID
          & obj.ba
          & obj.bg
          & obj.triangulatedPointsPosition
          & obj.trackedPoints
          & obj.triangulatedPoints
          & obj.validDbFeaturePoints
          & obj.invalidDbFeaturePoints
          & obj.bDBLost;

}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageStatus_t data.
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
void serialize(Seria &seria, MessageStatus_t &obj)
{
    uint8_t locStatusEvent = static_cast<uint8_t>(obj.event);

    seria & obj.frameID
          & locStatusEvent
          & obj.eventReason;

    obj.event = static_cast<LOC_EVENT_TYPE_E>(locStatusEvent);
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageFrameInfo_t data.
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
void serialize(Seria &seria, MessageFrameInfo_t &obj)
{
    seria & obj.frameID
          & obj.divisionIDs;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageDebugInfo_t data.
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
void serialize(Seria &seria, MessageDebugInfo_t &obj)
{
    uint8_t type = static_cast<uint8_t>(obj.type);
    
    seria & type
          & obj.moduleName
          & obj.mainTitle
          & obj.subTitle;

    obj.type = static_cast<VIS_DEBUG_TYPE>(type);
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageSensorStatus_t data.
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
void serialize(Seria &seria, MessageSensorStatus_t &obj)
{
    seria & obj.imgMissCount
          & obj.gpsMissCount
          & obj.imuMissCount
          & obj.obdMissCount;

}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize MessageCurrentPos_t data.
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
void serialize(Seria &seria, MessageCurrentPos_t &obj)
{
    int32_t type = static_cast<int32_t>(obj.compensateType);
    
    seria & obj.frmIdx
          & obj.taggedTs
          & obj.nonCompensatedLongtitude
          & obj.nonCompensatedLantitude
          & obj.nonCompensatedAltitude
          & obj.nonCompensatedHeading
          & type
          & obj.reportTs
          & obj.compensatedLongtitude
          & obj.compensatedLantitude
          & obj.compensatedAltitude
          & obj.compensatedHeading;

    obj.compensateType = static_cast<COMPENSATE_TYPE_E>(type);
}



}//! rdbSerializaion

}//！ RoadDBCore

#endif //！SERIAL_DRAW_MSG__H__
