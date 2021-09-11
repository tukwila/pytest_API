/**
 *******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SSeriaImpLandmark.h
 * @brief  Implementation of data serialization and deserialization for
 *         landmark snippet
 *******************************************************************************
 */

#include "CommunicateDef/RdbV2SCommon.h"
#include "CommunicateDef/RdbV2SLandmark.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpSlam.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"

#ifndef RDB_V2S_SERIA_IMP_LANDMARK_H
#define RDB_V2S_SERIA_IMP_LANDMARK_H

namespace roadDBCore

{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<LmVolume_t> data.
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
void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<LmVolume_t> &obj);

void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<LmVolume_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmBox_t data.
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
void serialize(Seria &seria, LmBox_t &obj)
{
    seria & obj.width & obj.thickness & obj.orientation;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmCylinder_t data.
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
void serialize(Seria &seria, LmCylinder_t &obj)
{
    seria & obj.radius;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmExtrusion_t data.
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
void serialize(Seria &seria, LmExtrusion_t &obj)
{
    seria & obj.spBaseLine & obj.thickness;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmIrregularBody_t data.
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
void serialize(Seria &seria, LmIrregularBody_t &obj)
{
    seria & obj.eIrregularType & obj.vecVertexs;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LM_SHAPE_E data.
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
void serialize(RdbV2SBinSerializer &seria, LM_SHAPE_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LM_SHAPE_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmVoxel_t data.
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
void serialize(Seria &seria, LmVoxel_t &obj)
{
    seria & obj.gravityCenter
          & obj.geoConf
          & obj.sdf
          & obj.subSemanticType
          & obj.subSemanticConf
          & obj.vecFrmIdx
          & obj.vecObservers
          & obj.roadPlaneFrmIdx;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmRoadPlane_t data.
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
void serialize(Seria &seria, LmRoadPlane_t &obj)
{
    seria & obj.normal
          & obj.height;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize TVM_DRIVER_VIEW_LABEL_E data.
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
void serialize(RdbV2SBinSerializer &seria, TVM_DRIVER_VIEW_LABEL_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, TVM_DRIVER_VIEW_LABEL_E &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmObject_t data.
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
void serialize(Seria &seria, LmObject_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, std::shared_ptr<LmVoxel_t>> cvecSpVoxels(obj.vecSpVoxels);

    seria & obj.id
          & obj.picName
          & obj.semanticType
          & obj.semanticConf
          & obj.subSemantic
          & obj.quality
          & obj.relRoadPlane
          & obj.vecRefPoints
          & obj.vecRefMapPtIdx
          & obj.vecFrmIdx
          & obj.vecSpLmVolumes
          & cvecSpVoxels
          & obj.voxelResolution
          & obj.vecVertexVoxelIdx
          & obj.colorSemanticInfo;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmSnippetSR_t data.
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
void serialize(Seria &seria, LmSnippetSR_t &obj)
{
    seria & obj.vecLmObjs
          & obj.vvecObjsLogic
          & obj.spSlamSnippet;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ModelConfigVehicle_t data.
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
void serialize(Seria &seria, ModelConfigVehicle_t &obj)
{
    seria & static_cast<ModelConfigSlam_t &>(obj)
          & obj.voxelScale;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize LmSnippetPayload_t data.
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
void serialize(RdbV2SBinSerializer &seria, LmSnippetPayload_t &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, LmSnippetPayload_t &obj);

}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_SERIA_IMP_LANDMARK_H
