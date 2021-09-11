/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2017-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbServerSeriaImpDB.h
 * @brief  Implementation of data serialization and deserialization for
 *         database structures in server side
 *******************************************************************************
 */

#include "algoInterface/IVehicleTransfer.h"
#include "algoInterface/IServerTransfer.h"
#include "algoInterface/IRoad.h"
#include "DBCommon/DBCommon.h"
#include "serialization/rdb/RdbSeriaCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"
#include "serialization/rdb/RdbV2SSeriaImpLandmark.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"

#ifndef RDB_SERVER_SERIA_IMP_DB_H
#define RDB_SERVER_SERIA_IMP_DB_H

namespace roadDBCore
{

namespace rdbSerialization
{

/*
  For Server DB Data:
*/
/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBObserver_t data.
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
void serialize(Seria &seria, DBObserver_t &obj)
{
    seria & obj.fnID
          & obj.points
          & obj.property
          & obj.descriptor;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBPoint_t data.
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
void serialize(Seria &seria, DBPoint_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, DBObserver_t> cvecObservers(obj.observers);

    seria & obj.nID
          & obj.ePointType
          & cvecObservers
          & obj.pos
          & obj.confidence
          & obj.mSemanticInfo;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize InlinerMpsInfo_t data.
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
void serialize(Seria &seria, InlinerMpsInfo_t &obj)
{
    seria & obj.ref1Id
          & obj.ref2Id
          & obj.mpInlinerIds;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBFrame_t data.
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
void serialize(Seria &seria, DBFrame_t &obj)
{
    seria & obj.nID
          & obj.bKF
          & obj.nBatchID
          & obj.coordinate
          & obj.rotation
          & obj.translation
          & obj.confidence
          & obj.oriFIdx
          & obj.coverageRatio;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBVoxelPoint data.
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
void serialize(Seria &seria, DBVoxelPoint &obj)
{
    seria & obj.geoConf
          & obj.tsdf
          & obj.hostKfNID
          & static_cast<DBPoint_t&>(obj);
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBPaintPoint data.
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
void serialize(Seria &seria, DBPaintPoint &obj)
{
    seria & obj.geoConf
          & obj.hostKfNID
          & static_cast<DBPoint_t&>(obj);
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize std::shared_ptr<DBPoint_t> data.
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
void serialize(RdbV2SBinDeserializer &seria, std::shared_ptr<DBPoint_t> &obj);

void serialize(RdbV2SBinSerializer &seria, std::shared_ptr<DBPoint_t> &obj);

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBFramesInfo_t data.
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
void serialize(Seria &seria, DBFramesInfo_t &obj)
{
    seria & obj.vecFrames;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBPointsInfo_t data.
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
void serialize(Seria &seria, DBPointsInfo_t &obj)
{
    seria & obj.vecMps;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBObject_t data.
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
void serialize(Seria &seria, DBObject_t &obj)
{
    seria & obj.nID
          & obj.semanticType
          & obj.subdividedSemantic
          & obj.semanticConf
          & obj.colorType
          & obj.objectSemantic
          & obj.cutSegIdx
          & obj.vecSpVoxelPoint
          & obj.relRoadPlane
          & obj.infoFromVolume_;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBObjectsInfo_t data.
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
void serialize(Seria &seria, DBObjectsInfo_t &obj)
{
    seria & obj.vecObjects
          & obj.vvecObjsLogic;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DB_REF_ATTR_E data.
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
void serialize(RdbV2SBinSerializer &seria, DB_REF_ATTR_E &obj);

template<>
void serialize(RdbV2SBinDeserializer &seria, DB_REF_ATTR_E &obj);

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
void serialize(Seria &seria, ModelConfigServer_t &obj)
{
    seria & static_cast<ModelConfigVehicle_t &>(obj)
          & obj.rtvName;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize DBServerReference_t data.
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
void serialize(Seria &seria, DBServerReference_t &obj)
{
    seria & obj.dbID
          & obj.originalID
          & obj.divisionID
          & obj.confidence
          & obj.version
          & obj.attribute
          & obj.optTimes
          & obj.oriRoadPlane
          & obj.config
          & obj.vecCutSegKfIdx
          & obj.spFramesInfo
          & obj.spPointsInfo
          & obj.spObjectsInfo;
}

///////////////////////////////////////////////////////////////////////////

/*
  For Landmark DB Data:
*/

template<typename Seria>
void serialize(Seria &seria, std::tuple<Point3d_t, Point3d_t, uint8_t> &obj)
{
    seria & std::get<0>(obj)
          & std::get<1>(obj)
          & std::get<2>(obj);
}

///////////////////////////////////////////////////////////////////////////

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
void serialize(Seria &seria, VehicleMapPoint_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint32_t> cvecObservers(obj.observers);
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, uint8_t> cvecDescriptors(obj.vecDescriptors);

    seria & obj.nID
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
void serialize(Seria &seria, VehicleKeyFrame_t &obj)
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
void serialize(Seria &seria, VehicleReference_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, VehicleKeyFrame_t> cvecKfs(obj.vecKfs);
    CodingVector<VECTOR_CODE_TYPE_3BYTE_E, VehicleMapPoint_t> cvecMps(obj.vecMps);

    seria & obj.dbID
          & cvecKfs
          & cvecMps
          & obj.vecObjects
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
void serialize(Seria &seria, VehicleNode_t &obj)
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
void serialize(Seria &seria, GpsItem_t &obj)
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
void serialize(Seria &seria, VehicleDivision_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_2BYTE_E, GpsItem_t> cvecGPS(obj.vecGPSTrajectory);

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
void serialize(Seria &seria, VehicleDivisionDetail_t &obj)
{
    CodingVector<VECTOR_CODE_TYPE_1BYTE_E, std::shared_ptr<VehicleReference_t>> cvecRef(obj.spReferences);

    for (const auto & reference: obj.spReferences)
    {
        obj.refConfidence[reference->dbID] = reference->confidence;
    }

    seria & obj.spDivision
          & obj.refConfidence;

    if (VEHICLE_DIV_MODE_NO_REF_E != obj.eMode)
    {
        CodingVector<VECTOR_CODE_TYPE_1BYTE_E, std::shared_ptr<VehicleReference_t>> cvecRef(obj.spReferences);

        seria & cvecRef;
    }
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleObject_t data.
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
void serialize(Seria &seria, VehicleObject_t &obj)
{
    seria & obj.nID
          & obj.semanticType
          & obj.subdividedSemantic
          & obj.vecPoints;
}

/**
 *******************************************************************************
 * @brief serialize - Serialize or deserialize VehicleVoxeObject data.
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
void serialize(Seria &seria, VehicleVoxelObject_t &obj)
{
    seria & obj.nID
          & obj.semanticType
          & obj.vecPoints;
}

}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_SERVER_SERIA_IMP_DB_H



