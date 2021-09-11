/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2017-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SSeriaImpSlam.h
 * @brief  Implementation of data serialization and deserialization for
 *         slam snippet
 *******************************************************************************
 */

#include "CommunicateDef/RdbV2SCommon.h"
#include "CommunicateDef/RdbV2SSlam.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"

#ifndef RDB_V2S_SERIA_IMP_SLAM_H
#define RDB_V2S_SERIA_IMP_SLAM_H

namespace roadDBCore

{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Ref3DLmPointSR_t data.
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
void serialize(Seria &seria, Ref3DLmPointSR_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint16_t> cvecObsFrmIdx(obj.vecObsFrmIdx);

    seria & cvecObsFrmIdx;

    CodingVector<VECTOR_CODE_TYPE_0BYTE_E, Point2f_t> cvecKeyPoints(obj.vec2DPoints, obj.vecObsFrmIdx.size());
    CodingVector<VECTOR_CODE_TYPE_0BYTE_E, uint8_t> cvecDescID(obj.vecDescID, obj.vecObsFrmIdx.size());
    CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, uint8_t> cvecDescriptors(obj.vecDescriptors);

    seria & cvecKeyPoints
          & cvecDescID
          & cvecDescriptors
          & obj.semanticContext
          & obj.p3dID;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize Inc3DLmPointSR_t data.
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
void serialize(Seria &seria, Inc3DLmPointSR_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint16_t> cvecObsFrmIdx(obj.vecObsFrmIdx);

    seria & cvecObsFrmIdx;

    CodingVector<VECTOR_CODE_TYPE_0BYTE_E, Point2f_t> cvecKeyPoints(obj.vec2DPoints, obj.vecObsFrmIdx.size());
    CodingVector<VECTOR_CODE_TYPE_0BYTE_E, uint8_t> cvecDescID(obj.vecDescID, obj.vecObsFrmIdx.size());
    CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, uint8_t> cvecDescriptors(obj.vecDescriptors);

    seria & cvecKeyPoints
          & cvecDescID
          & cvecDescriptors
          & obj.semanticContext
          & obj.position;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize KeyFrameSR_t data.
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
void serialize(Seria &seria, KeyFrameSR_t &obj)
{
    seria & obj.id
          & obj.bKF
          & obj.rotation
          & obj.translation
          & obj.relativeGPS
          & obj.refTimestamp
          & obj.coverageRatio;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SlamSnippetSR_t data.
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
void serialize(Seria &seria, SlamSnippetSR_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, KeyFrameSR_t>        cvecKeyFrames(obj.vecKeyFrames);
    CodingVector<VECTOR_CODE_TYPE_3BYTE_E, Inc3DLmPointSR_t>    cvec3DPoints(obj.vec3DPoints);

    seria & cvecKeyFrames
          & cvec3DPoints;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ModelConfigSlam_t data.
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
void serialize(Seria &seria, ModelConfigSlam_t &obj)
{
    seria & static_cast<ModelConfigBase_t &>(obj)
          & obj.hoodHeight;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize SlamSnippetPayload_t data.
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
void serialize(RdbV2SBinSerializer &seria, SlamSnippetPayload_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, SlamSnippetPayload_t &obj);


}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_SERIA_IMP_SLAM_H
