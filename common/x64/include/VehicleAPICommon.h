/**
 *******************************************************************************
 *                               RoadDB Confidential
 *                         Copyright (c) RoadDB 2019-2020
 *
 *           This software is furnished under license and may be used or
 *           copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file	VehicleAPICommon.h
 * @brief  	This file defines RoadDB common definitions.
 *******************************************************************************
 */

#pragma once

#include <stdint.h>
#include <memory>
#include <unordered_map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <unordered_set>
#include "typeDef.h"
// #include "CommunicateDef/RdbV2SGeometry.h"
#include "algoInterface/IVehicleTransfer.h"

using namespace std;
namespace RDBVehicleAPI
{
#define OUT
#define IN

typedef float float32_t;
typedef double float64_t;
// typedef uint64_t objectID_t;
//typedef std::string objectID_t;
typedef uint32_t errCode_t;

#define CONTACTER_OF_CURVE_ID "*"
#define DB_VEHICLE_SUCCESS 0
#define DB_VEHICLE_SUBCODE_SUCCESS 0

#define DB_VEHICLE_ILLEGAL_SPEED_LIMIT 0xffffffff

// #define MODAL_FRIEND_CLASS  BaseRoadDataInfo

// #define MODAL_FRIEND_CLASS  BaseRoadDataInfo

#define FRIEND_2_ROADDATAINFO        \
    friend class BaseRoadDataInfo;   \
    friend class SqliteRoadDataInfo; \
    friend class PBRoadDataInfo;

#define DELT (1e-6)

#define RETURN_IFNULL(ptrName) \
    if (ptrName == nullptr)    \
    {                          \
        return;                \
    }

#define CONTINUE_IFNULL(ptrName) \
    if (ptrName == nullptr)      \
    {                            \
        continue;                \
    }

#define runAssert(exp) assert(exp)

#define PrintSysErr "|error|" << errno << "|strerror|" << strerror(errno)

#define VSPVec(T) std::vector<std::shared_ptr<const T>>
#define VWPVec(T) std::vector<std::weak_ptr<const T>>

#define SP(T) std::shared_ptr<T>
#define PTR_CONVERT(T) std::dynamic_pointer_cast<const T>
#define CONST_CONVERT(T) std::const_pointer_cast<T>

#define VSPVec(T) std::vector<std::shared_ptr<const T>>
#define VWPVec(T) std::vector<std::weak_ptr<const T>>

#define DEFAULT_LINEINDEX -1
#define DEFAULT_LINEID ""
#define DEFAULT_LANEID ""

#define ROUNDINGDIGITS 2
#define ROUNDING(t) ((floor(t * pow(10, ROUNDINGDIGITS))) / (pow(10, ROUNDINGDIGITS)))

typedef struct
{
    uint16_t platformID;
    uint8_t seqNum;
    uint8_t requestType;
    uint16_t majorVersion;
    uint16_t minorVersion;
    uint32_t length;
} t_ReqHeader;

typedef struct
{
    uint16_t reserved;
    uint8_t seqNum;
    uint8_t responseType;
    uint16_t majorVersion;
    uint16_t minorVersion;
    uint32_t length;
} t_ResHeader;

enum TransportType_t
{
    E_Transport_Type_TCP,
    E_Transport_Type_UDP
};

typedef struct t_SocketInfo
{
    int sockfd;
    struct sockaddr_in clientAddr;
    socklen_t clientSockLen;
    TransportType_t transportType;

    // t_SocketInfo() { transportType = E_Transport_Type_UDP; }
} t_SocketInfo;

typedef roadDBCore::SegmentID_t segmentID_t;
typedef std::vector<segmentID_t> segmentIDSeq_t;
typedef std::set<segmentID_t> segmentIDSet_t;

typedef uint64_t divisionID_t;
typedef uint64_t nodeID_t;
typedef std::vector<divisionID_t> divisionIDSeq_t;
typedef std::set<divisionID_t> divisionIDSet_t;

#define TO_STR(X) (boost::lexical_cast<std::string>(X))

struct point3D_t
{
    point3D_t() : x(0), y(0), z(0) {}
    point3D_t(float64_t _x, float64_t _y, float64_t _z) : x(_x), y(_y), z(_z) {}
    float64_t x = 0;
    float64_t y = 0;
    float64_t z = 0;
};

struct WGS84_t
{
    WGS84_t() : lon(0), lat(0), alt(0) {}
    WGS84_t(float64_t _lon, float64_t _lat, float64_t _alt) : lon(_lon), lat(_lat), alt(_alt) {}
    float64_t lon = 0;
    float64_t lat = 0;
    float64_t alt = 0;
};

struct NDSPoint_t
{
    NDSPoint_t() : lon(0), lat(0), alt(0) {}
    NDSPoint_t(int32_t _lon, int32_t _lat, int32_t _alt) : lon(_lon), lat(_lat), alt(_alt) {}
    int32_t lon = 0;
    int32_t lat = 0;
    int32_t alt = 0;
};

struct Pose_t
{
    Pose_t() : pitch(0), yaw(0), roll(0) {}
    Pose_t(float64_t _pitch, float64_t _yaw, float64_t _roll) : pitch(_pitch), yaw(_yaw), roll(_roll) {}
    float64_t pitch = 0.0;
    float64_t yaw = 0.0;
    float64_t roll = 0.0;
};

typedef std::unordered_set<segmentID_t> segmentIDUnorderSet_t;

enum CACHE_MODE_E
{
    CACHE_MODE_WHOLE_E = 1,
    CACHE_MODE_REALTIME_E = 2,
};
typedef int32_t layerIndex_t;

/*[5][4][3][2][1][Center=0][1][2][3][4][5]*/
struct refreshConf_t
{
    // used in realtime mode
    layerIndex_t cacheLayerCount = 5;      // How many layers to cache while recaching everytime
    layerIndex_t minRefreshLayerIndex = 2; // Minium layer index(If the layer index of reported segent >= this value, do recache)
    uint64_t gpsCheckTimeInterval = 1000;  // used in realtime mode. unit: ms
};

struct threadAttribute_t
{
    int32_t schedPolicy = SCHED_OTHER;
    int32_t priority = 0;
    std::vector<int32_t> vCpus;
};

enum EXCEPTION_DATA_HANDLE_MODE
{
    EDH_MODE_ACCEPTED = 0, // Compitable with exception data.
    EDH_MODE_REFUSED = 1   // Exception data is't accepted by strict checking.
};

enum EHORIZON_DETAIL_LEVEL
{
    EHORIZON_DETAIL_LEVEL_BRIEF = 1, // without profile info
    EHORIZON_DETAIL_LEVEL_DETAIL = 2 // with profile info
};

enum ECURRENT_LANE_DATA_TYPE : uint32_t
{
    ECLane_None = 0x0,
    ECLane_Curvilinear_Left_LaneMarker = 0x1,
    ECLane_Curvilinear_Right_LaneMarker = 0x2,
    ECLane_Curvilinear_Slope = 0x4,
    ECLane_Curvilinear_Camber = 0x8,
    ECLane_Curveilinear_LaneMarker = 0xF,
    ECLane_EVPTrajectory = 0x10,
    ECLane_MiddleLineTrajectory = 0x20,
    ECLane_All_Trajectory = 0x30,
    ECLane_SpeedLimit = 0x40,
    ECLane_NumberOfLanes = 0x80,
    ECLane_Linear_Left_LaneMarker = 0x100,
    ECLane_Linear_Right_LaneMarker = 0x200,
    ECLane_Linear_Slope = 0x400,
    ECLane_Linear_Camber = 0x800,
    ECLane_Linear_LaneMarker = 0xF00,
    ECLane_Linear_LateralDistance = 0x1000,
    ECLane_All = 0x1FFF
};
} // namespace RDBVehicleAPI
