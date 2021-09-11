/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbSeriaImpVDB_40007.h
 * @brief  Implementation of data serialization and deserialization for
 *         database structures in server side
 *******************************************************************************
 */

#include "serialization/vdbCompatibility/vehicleData_40007.h"
#include "DBCommon/DBCommon.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"
#include "serialization/rdb/RdbV2SSeriaImpLandmark.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"

#ifndef RDB_SERIA_IMP_VDB_40007_H
#define RDB_SERIA_IMP_VDB_40007_H

namespace roadDBCore
{
namespace rdbSerialization
{
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
void serialize(Seria &seria, vdb_40007::VehicleMapPoint_t &obj)
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
void serialize(Seria &seria, vdb_40007::VehicleKeyFrame_t &obj)
{
    seria & obj.nID
          & obj.coordinate
          & obj.quaternion
          & obj.translation
          & obj.batchID
          & obj.oriFIdx
          & obj.fx
          & obj.fy
          & obj.cx
          & obj.cy;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize BatchInfo_t data.
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
void serialize(Seria &seria, vdb_40007::BatchInfo_t &obj)
{
    seria & obj.rtvname;
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
void serialize(Seria &seria, vdb_40007::VehicleReference_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, vdb_40007::VehicleKeyFrame_t> cvecKfs(obj.vecKfs);
    CodingVector<VECTOR_CODE_TYPE_3BYTE_E, vdb_40007::VehicleMapPoint_t> cvecMps(obj.vecMps);

    seria & obj.dbID & cvecKfs & cvecMps
          & obj.confidence
          & obj.version & obj.descriptorType
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
void serialize(Seria &seria, vdb_40007::VehicleNode_t &obj)
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
void serialize(Seria &seria, vdb_40007::GpsItem_t &obj)
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
void serialize(Seria &seria, vdb_40007::VehicleDivision_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, vdb_40007::GpsItem_t> cvecGPS(obj.vecGPSTrajectory);

    seria & obj.dbID
          & obj.spFromNode->dbID
          & obj.spToNode->dbID
          & obj.trajectoryType
          & cvecGPS
          & obj.rightNeighbours
          & obj.leftNeighbours
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
void serialize(RdbV2SBinSerializer &seria, vdb_40007::VehicleDivisionDetail_t &obj);

void serialize(RdbV2SBinDeserializer &seria, vdb_40007::VehicleDivisionDetail_t &obj);




} // namespace rdbSerialization

} // namespace roadDBCore




#endif //RDB_SERIA_IMP_VDB_40007_H



