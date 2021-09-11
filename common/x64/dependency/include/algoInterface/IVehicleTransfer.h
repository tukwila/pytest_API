/**
 *******************************************************************************
 *                          RoadDB Confidential
 *         Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IVehicleTransfer.h
 * @brief  Head file of class ITransfer which define the interfaces provided by
 *            transfer in server.
 *******************************************************************************
 */
#ifndef IVEHICLE_TRANSTER_H
#define IVEHICLE_TRANSTER_H
#include <string>
#include <vector>
#include <map>
#include <list>
#include <limits> // numeric_limits
#include "typeDef.h"
#include "segment/Segment.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "DBCommon/DBCommon.h"

//declare the two class which is defined in algo


namespace roadDBCore
{
typedef uint64_t    DivisionID_t;
typedef uint64_t    NodeID_t;

enum VEHICLE_DIV_MODE_E : uint8_t
{
    VEHICLE_DIV_MODE_WHOLE_E = 0,    // The whole reference (General info + KF_MP + Landmark objects)
    VEHICLE_DIV_MODE_GENERAL_E,      // Only General info of reference
    VEHICLE_DIV_MODE_NO_REF_E,       // There is no reference info in division
    VEHICLE_DIV_MAX_E
};

struct VehicleMapPoint_t  // with 3D descriptor
{
    uint32_t                nID = 0;

    /* Start: Add for localizaton: Don't serialize for them  */
    uint32_t                locID = 0;
    bool                    locIsBad = false;
    uint32_t                locKfCount = 0; //mean the kf selected
    /* End: Just add for localizaton. Don't serialize for them */

    /* Code 2-byte vector size for serialization */
    std::vector<uint32_t>   observers; //kf id
    Point3f_t               pos;

    /* Code 2-byte vector size for serialization */
    std::vector<uint8_t>    vecDescriptors;

    float32_t               confidence = 0.0F;

    std::map<uint8_t, uint8_t>  mSemanticInfo;   // semantic type and confidence
};

struct VehicleKeyFrame_t
{
    uint32_t    nID{0};

    /* Start: Just add for localizaton. Don't serialize for them */
    uint32_t    locID = 0;
    uint64_t    locDivisionID;
    Point3d_t   locCoordinate;
    std::vector<std::shared_ptr<VehicleMapPoint_t>> locVspMps;
    /* End: Just add for localizaton. Don't serialize for them */

    Point3f_t   relativeGPS; // originally, it's named as coordinate.
    float32_t   quaternion[4]{0};
    float32_t   translation[3]{0};
    uint64_t    batchID{0};

    /* the original index of frame in rtv which generates this key frame */
    int32_t     oriFIdx{-1};
};

struct VehicleObject_t  // with 3D Object
{
    uint32_t                nID = 0;

    //semanticType subdividedSemantic from snippet
    uint8_t                 semanticType = 0;
    uint32_t                subdividedSemantic = 0;

    //it is gravity center for paint point and voxel point
    std::vector<Point3f_t>  vecPoints;
};

struct VehicleReference_t
{
    uint64_t                        dbID = 0;

    /* Code 2-byte vector size for serialization, supports up to 65535 key frames */
    std::vector<VehicleKeyFrame_t>  vecKfs;   //serilization

    /* Code 3-byte vector size for serialization, supports up to 65535 * 256 landmark points */
    std::vector<VehicleMapPoint_t>  vecMps;   //serilization

    std::vector<VehicleObject_t>    vecObjects;   //serilization

    float32_t                       confidence = 0.0F;
    uint64_t                        version = 0LL;

    /* The detail batch info for specified batch. Key is batch ID. */
    std::map<uint64_t, ModelConfigServer_t> mapBatchInfo;
};

struct VehicleNode_t
{
    uint64_t                  dbID = 0;
    Point3f_t                 coordinate;
};

struct GpsItem_t
{
    Point3d_t gps;
    uint8_t   quality = 0;

    GpsItem_t(const Point3d_t &gpsIn = Point3d_t(), uint8_t qualityIn = 0):
                gps(gpsIn), quality(qualityIn) {}
};

struct VehicleDivision_t
{
    uint64_t                       dbID = 0LL;     //generate by database;
    std::shared_ptr<VehicleNode_t> spFromNode;
    std::shared_ptr<VehicleNode_t> spToNode;

    /* Start: Just add for localizaton. Don't serialize for them */
    uint32_t                       locID = 0;
    std::vector<uint64_t>          locNextDivIDs;
    std::vector<std::shared_ptr<VehicleDivision_t>> locNextDivisions;
    /* End: Just add for localizaton. Don't serialize for them */

    /* 1: trajectory in form of GPS points; 0: for no gps trajectory */
    uint16_t                       trajectoryType = 0;

    /* Code 2-byte vector size for GPS Trajectory , supports up to 65536 Trajectories */
    std::vector<GpsItem_t>         vecGPSTrajectory;

    //Passed Segments calculate according to vecGPSTrajectory
    std::set<SegmentID_t>          passSegIDs;
};

struct VehicleDivisionDetail_t //ServerSecMap
{
    std::shared_ptr<VehicleDivision_t>       spDivision;
    std::map<uint64_t, float32_t>            refConfidence; //key: referenceID;  value: confidence

    /* Code 1-byte */
    std::vector<std::shared_ptr<VehicleReference_t>>  spReferences;

    /* Don't serialize this field */
    VEHICLE_DIV_MODE_E                       eMode = VEHICLE_DIV_MODE_WHOLE_E;
};

struct VehicleVoxelObject_t
{
    uint64_t  nID = 0LL;
    uint8_t   semanticType = 0;

    std::vector<roadDBCore::Point3f_t>  vecPoints;
};

typedef std::vector<std::shared_ptr<VehicleDivisionDetail_t>> VecVehicleDivision_t;

template<>
struct VehicleDBData<DB_VERSION_VEHICLE_FILE>
{
    using DATA_TYPE = VehicleDivisionDetail_t;
    static const bool bSupport = true;
};

class IVehicleTransfer
{
public:
    virtual ~IVehicleTransfer(){};

    virtual void setRefSegId(SegmentID_t segId) = 0;
    virtual void getRefSegId(SegmentID_t &segId) = 0;
    virtual uint32_t putDivisions(VecVehicleDivision_t &spDivisions) = 0;

    virtual uint32_t getDivisions(const std::set<SegmentID_t> &segIDs,
                                  VecVehicleDivision_t &spDivisions,
                                  VEHICLE_DIV_MODE_E eMode = VEHICLE_DIV_MODE_GENERAL_E) = 0;

    virtual uint32_t getDivisions(const std::vector<uint64_t> &divisionIDs,
                                  VecVehicleDivision_t &spDivisions) = 0;

    virtual uint32_t getDivision(DivisionID_t divisionID,
                                 std::shared_ptr<VehicleDivisionDetail_t> &spDivision) = 0;
};


}




#endif
