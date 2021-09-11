/**
 *******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbSeriaImpVDB_40009.h
 * @brief  Implementation of data serialization and deserialization for
 *         database structures in server side
 *******************************************************************************
 */

#include "serialization/vdbCompatibility/vehicleData_40009.h"
#include "DBCommon/DBCommon.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"
#include "serialization/rdb/RdbV2SSeriaImpLandmark.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"

#ifndef RDB_SERIA_IMP_VDB_40009_H
#define RDB_SERIA_IMP_VDB_40009_H

namespace roadDBCore
{
namespace rdbSerialization
{
/*
  For ModelConfigServer_t Data:
*/

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
void serialize(Seria &seria, vdb_40009::ModelConfigBase_t &obj)
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
          & obj.baseTimestamp;
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
void serialize(Seria &seria, vdb_40009::ModelConfigSlam_t &obj)
{
    seria & static_cast<vdb_40009::ModelConfigBase_t &>(obj)
          & obj.hoodHeight;
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
void serialize(Seria &seria, vdb_40009::ModelConfigVehicle_t &obj)
{
    seria & static_cast<vdb_40009::ModelConfigBase_t &>(obj)
          & obj.voxelScale;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize ModelConfigServer_t data.
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
void serialize(Seria &seria, vdb_40009::ModelConfigServer_t &obj)
{
    seria & static_cast<vdb_40009::ModelConfigVehicle_t &>(obj)
          & obj.rtvName;
}

/*
  For Vehicle DB Data:
*/

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleMapPoint_t data.
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
void serialize(Seria &seria, vdb_40009::VehicleMapPoint_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint32_t> cvecObservers(obj.observers);
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint8_t> cvecDescriptors(obj.vecDescriptors);

    seria & obj.nID
          & obj.ePointType
          & cvecObservers
          & obj.pos
          & cvecDescriptors
          & obj.confidence
          & obj.mSemanticInfo;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleKeyFrame_t data.
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
void serialize(Seria &seria, vdb_40009::VehicleKeyFrame_t &obj)
{
    seria & obj.nID
          & obj.relativeGPS
          & obj.quaternion
          & obj.translation
          & obj.batchID
          & obj.oriFIdx;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleReference data.
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
void serialize(Seria &seria, vdb_40009::VehicleReference_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, vdb_40009::VehicleKeyFrame_t> cvecKfs(obj.vecKfs);
    CodingVector<VECTOR_CODE_TYPE_3BYTE_E, vdb_40009::VehicleMapPoint_t> cvecMps(obj.vecMps);

    seria & obj.dbID
          & cvecKfs
          & cvecMps
          & obj.confidence
          & obj.version
          & obj.mapBatchInfo;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleNode data.
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
void serialize(Seria &seria, vdb_40009::VehicleNode_t &obj)
{
    seria & obj.dbID
          & obj.coordinate;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize GpsItem_t data.
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
void serialize(Seria &seria, vdb_40009::GpsItem_t &obj)
{
    seria & obj.quality
          & obj.gps;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleSection data.
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
void serialize(Seria &seria, vdb_40009::VehicleDivision_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, vdb_40009::GpsItem_t> cvecGPS(obj.vecGPSTrajectory);

    seria & obj.dbID
          & obj.spFromNode
          & obj.spToNode
          & obj.trajectoryType
          & cvecGPS
          & obj.passSegIDs;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleSectionDetail data.
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
void serialize(Seria &seria, vdb_40009::VehicleDivisionDetail_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_1BYTE_E, std::shared_ptr<vdb_40009::VehicleReference_t>> cvecRef(obj.spReferences);

    for (const auto & reference: obj.spReferences)
    {
        obj.refConfidence[reference->dbID] = reference->confidence;
    }

    seria & obj.spDivision
          & obj.refConfidence;

    if (vdb_40009::VEHICLE_DIV_MODE_NO_REF_E != obj.eMode)
    {
        CodingVector<VECTOR_CODE_TYPE_1BYTE_E, std::shared_ptr<vdb_40009::VehicleReference_t>> cvecRef(obj.spReferences);

        seria & cvecRef;
    }
}

} // namespace rdbSerialization

} // namespace roadDBCore




#endif //RDB_SERIA_IMP_VDB_40009_H



