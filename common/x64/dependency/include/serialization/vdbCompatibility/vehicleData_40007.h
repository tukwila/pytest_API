/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   vehicleData_40007.h
 * @brief  Head file of vehicle data structures and interface for version 0x40007.
 *******************************************************************************
 */

#ifndef VEHICLE_DATA_40007_H
#define VEHICLE_DATA_40007_H
#include <string>
#include <vector>
#include <map>
#include <list>
#include <limits> // numeric_limits
#include "typeDef.h"
#include "segment/Segment.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "DBCommon/DBCommon.h"
#include "algoInterface/IVehicleTransfer.h"


//declare the two class which is defined in algo


namespace roadDBCore
{
namespace rdbSerialization
{
namespace vdb_40007
{
const uint32_t DB_VERSION_VEHICLE_FILE = RDB_VERSION_MAIN + 7;

typedef uint64_t    DivisionID_t;
typedef uint64_t    NodeID_t;

enum VEHICLE_DIV_MODE_E : uint8_t
{
    VEHICLE_DIV_MODE_WHOLE_E = 0,    // The whole reference (General info + KF_MP + Landmark objects)
    VEHICLE_DIV_MODE_GENERAL_E,      // Only General info of reference
    VEHICLE_DIV_MODE_NO_REF_E,       // There is no reference info in division
    VEHICLE_DIV_MAX_E
};

struct VehicleKeyFrame_t
{
    uint32_t    nID{0};

    Point3f_t   coordinate;
    float32_t   quaternion[4]{0};
    float32_t   translation[3]{0};
    uint64_t    batchID{0};

    /* the original index of frame in rtv which generates this key frame */
    int32_t     oriFIdx{-1};

    /*
     The correctly scaled camera intrinsics which generate the key frame,
     which is good for re-projecting mappoints
    */
    float32_t   fx{0};
    float32_t   fy{0};
    float32_t   cx{0}; // optical center in x
    float32_t   cy{0}; // optical center in y
};

struct VehicleMapPoint_t  // with 3D descriptor
{
    uint32_t                nID = 0;

    // 0: mappoint; 1: voxel point; 2: paint point
    DB_POINT_TYPE_E         ePointType = DB_POINT_TYPE_MAX_E;

    /* Code 2-byte vector size for serialization */
    std::vector<uint32_t>   observers; //kf id
    Point3f_t               pos;

    /* Code 2-byte vector size for serialization */
    std::vector<uint8_t>    vecDescriptors;

    float32_t               confidence = 0.0F;

    std::map<uint8_t, uint8_t>  mSemanticInfo;   // semantic type and confidence
};

struct BatchInfo_t
{
    std::string rtvname;
};

struct VehicleReference_t
{
    uint64_t                        dbID;

    /* Code 2-byte vector size for serialization, supports up to 65535 key frames */
    std::vector<VehicleKeyFrame_t>  vecKfs;   //serilization

    /* Code 3-byte vector size for serialization, supports up to 65535 * 256 landmark points */
    std::vector<VehicleMapPoint_t>  vecMps;   //serilization

    float32_t                       confidence;
    uint64_t                        version;
    uint32_t                        descriptorType;

    /* the detail batch info for specified batch.
       Key is batch ID.
    */
    std::map<uint64_t, BatchInfo_t> mapBatchInfo;

    VehicleReference_t() : dbID(0),
                         confidence(0),
                         version(0),
                         descriptorType(0)
    {}
};

struct VehicleNode_t
{
    uint64_t                  dbID;
    Point3f_t                 coordinate;

    VehicleNode_t() : dbID(0) {}
};

struct GpsItem_t
{
    uint8_t   quality = 0;
    Point3d_t gps;
};

struct VehicleDivision_t
{
    uint64_t                       dbID = 0LL;     //generate by database;
    std::shared_ptr<VehicleNode_t> spFromNode;
    std::shared_ptr<VehicleNode_t> spToNode;

    /* 1: trajectory in form of GPS points; 0: for no gps trajectory */
    uint16_t                       trajectoryType = 0;

    /* Code 2-byte vector size for GPS Trajectory , supports up to 65536 Trajectories */
    std::vector<GpsItem_t>         vecGPSTrajectory;

    std::map<uint64_t, uint8_t>    rightNeighbours;
    std::map<uint64_t, uint8_t>    leftNeighbours;

    //Passed Segments calculate according to vecGPSTrajectory
    std::set<SegmentID_t>          passSegIDs;
};

struct VehicleDivisionDetail_t //ServerSecMap
{
    std::shared_ptr<VehicleDivision_t>       spDivision;
    std::map<uint64_t, float32_t>            refConfidence; //key: referenceID;  value: confidence

    VEHICLE_DIV_MODE_E                       eMode = VEHICLE_DIV_MODE_WHOLE_E;

    /* Code 1-byte */
    std::vector<std::shared_ptr<VehicleReference_t>>  spReferences;
};

typedef std::vector<std::shared_ptr<VehicleDivisionDetail_t>> VecVehicleDivision_t;



uint32_t convertCompatibleData(const VehicleReference_t &srcData,
                            roadDBCore::VehicleReference_t &dstData);

uint32_t convertCompatibleData(const VehicleDivision_t &srcData,
                            roadDBCore::VehicleDivision_t &dstData);

uint32_t convertCompatibleData(const VehicleDivisionDetail_t &srcData,
                            roadDBCore::VehicleDivisionDetail_t &dstData);

} // namespace vdb_40007
} // namespace rdbSerialization

template<>
struct VehicleDBData<rdbSerialization::vdb_40007::DB_VERSION_VEHICLE_FILE>
{
    using DATA_TYPE = rdbSerialization::vdb_40007::VehicleDivisionDetail_t;
    static const bool bSupport = true;
};

} // namespace roadDBCore




#endif
