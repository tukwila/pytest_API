/**
 ************************************************************************************
 *                          RoadDB Confidential
 *         Copyright (c) Continental AG. 2017-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   RdbV2SCommon.h
 * @brief  Definitions of common structures
 ************************************************************************************
 */
#ifndef RDB_V2S_COMMON_H
#define RDB_V2S_COMMON_H

#include <vector>       // std::vector
#include <memory>       // std::shared_ptr
#include <sstream>      // std::stringstream
#include <cmath>        // std::fabs
#include <string.h>     // memcpy
#include "typeDef.h"    // uint16 etc.

namespace roadDBCore
{
const uint32_t RDB_SLAM_SNIPPET_VERSION  = RDB_VERSION_MAIN + 0X0000000e;
const uint32_t RDB_ALIGN_SNIPPET_VERSION = RDB_VERSION_MAIN + 0X00000008;
const uint32_t RDB_SDOR_SNIPPET_VERSION  = RDB_VERSION_MAIN + 0X00000010;
const uint32_t RDB_LM_SNIPPET_VERSION    = RDB_VERSION_MAIN + 0X00000005;

enum SNIPPET_PAYLOAD_TYPE_E: uint8_t
{
    SNIPPET_PAYLOAD_TYPE_ROAD_E,        // RoadDatabase 1.0 road plane or 2.0 road object
    SNIPPET_PAYLOAD_TYPE_SLAM_E,        // RoadDatabase 2.0 and 3.0 slam snippet
    SNIPPET_PAYLOAD_TYPE_RT_E,          // RoadDatabase 2.0 and 3.0 RTMatrix
    SNIPPET_PAYLOAD_TYPE_LM_E,          // RoadDatabase 4.0 Landmark snippet
    SNIPPET_PAYLOAD_TYPE_MAX_E
};

enum LANE_MARKING_TYPE_E : uint8_t
{
    LANE_MARKING_TYPE_UNLABELED_E = 0,
    LANE_MARKING_TYPE_ROAD_SURFACE_E,
    LANE_MARKING_TYPE_OFF_ROAD_E,
    LANE_MARKING_TYPE_SOLID_LINE_E,
    LANE_MARKING_TYPE_DASHED_LINE_E,
    LANE_MARKING_TYPE_FORKED_LINE_E,
    LANE_MARKING_TYPE_MERGED_LINE_E,
    LANE_MARKING_TYPE_ROAD_MARKING_E,
    LANE_MARKING_TYPE_DASHED_DASHED_DASHED_LINE_E,
    LANE_MARKING_TYPE_DASHED_SOLID_LINE_E,
    LANE_MARKING_TYPE_SOLID_DASHED_LINE_E,
    LANE_MARKING_TYPE_DASHED_SOLID_DASHED_LINE_E,
    LANE_MARKING_TYPE_DOUBLE_SOLID_LINE_E,
    LANE_MARKING_TYPE_DOUBLE_DASHED_LINE_E,
    LANE_MARKING_TYPE_SINGLE_SOLID_LINE_E, //useless, for some history version
    LANE_MARKING_TYPE_MAX_E,
};

enum LANE_MARKING_COLOR_TYPE_E : uint8_t
{
    LANE_MARKING_COLOR_TYPE_UNLABELED_E = 0,
    LANE_MARKING_COLOR_TYPE_WHITE_E,
    LANE_MARKING_COLOR_TYPE_BLACK_E,
    LANE_MARKING_COLOR_TYPE_YELLOW_E,
    LANE_MARKING_COLOR_TYPE_WHITE_AND_WHITE_E,
    LANE_MARKING_COLOR_TYPE_BLACK_AND_BLACK_E,
    LANE_MARKING_COLOR_TYPE_YELLOW_AND_YELLOW_E,
    LANE_MARKING_COLOR_TYPE_WHITE_AND_WHITE_AND_WHITE_E,
    LANE_MARKING_COLOR_TYPE_WHITE_AND_YELLOW_AND_WHITE_E,
    LANE_MARKING_COLOR_TYPE_MAX_E,
};

enum ROAD_POINT_TYPE_E : uint8_t
{
    ROAD_POINT_TYPE_SOLID_DASHED_SPLITOR_E = 0,
    ROAD_POINT_TYPE_CROSS_WAY_E,
    ROAD_POINT_TYPE_MERGE_POINT_E,
    ROAD_POINT_TYPE_FORK_POINT_E,
    ROAD_POINT_TYPE_MAX_E,
};

// New descriptor type would be defined
enum SLAM_DESCRIPTOR_TYPE_E: uint8_t
{
    SLAM_DESCRIPTOR_TYPE_SGD_FLOAT32_256B_E = 0,
    SLAM_DESCRIPTOR_TYPE_SGD_64BYTE_1FLOAT_68B_E,
    SLAM_DESCRIPTOR_TYPE_SGD_48BYTE_1FLOAT_52B_E,
    SLAM_DESCRIPTOR_TYPE_SGD_4BIT_32B_E,
    SLAM_DESCRIPTOR_TYPE_SGD_24BYTE_3FLOAT_36B_E,
    SLAM_DESCRIPTOR_TYPE_MAX_E
};

const uint32_t DESCRIPTOR_GROUP_SIZE[SLAM_DESCRIPTOR_TYPE_MAX_E] = {256, 68, 52, 32, 36};

struct RoadDatabaseHead_t
{
    uint16_t                headerLen;           // Header length
    SNIPPET_PAYLOAD_TYPE_E  next;                // next payload type
    uint32_t                version;             // Header version
    uint64_t                refTimeStamp;        // Reference time stamp

    /* Convert from float64_t to uint32_t:
       1). Method: (Raw float64_t lon + 180) * (360 / 2^32) + 0.5
           Precision: 111314M * 360 / 2^32 = 111314M * 0.00000008381903171539306640625
                                           = 0.0093302316963672637939453125 M
           0.93302316963672637939453125 CM
       2). Method: Q 9-23 format
           Precision: 111314M * 1 / 2^23 = 111314M * 0.00000011920928955078125
                                         = 0.0132696628570556640625 M
           1.32696628570556640625 CM */
    uint32_t refGpsLon;           // Reference GPS lon

    /* Convert from float64_t to uint32_t:
       Method: (Raw float64_t lat + 90) * (180 / 2^32) + 0.5
       Precision: 110950M *180 / 2^32 = 110950M * 0.000000041909515857696533203125
       0.464986078441143035888671875 CM */
    uint32_t refGpsLat;           // Reference GPS lat.

    /* Convert from float64_t to uint32_t:
       Method: (Raw float64_t alt + 1000) * 1000 + 0.5
       Precision: 0.1 CM */
    uint32_t refGpsAlt;           // Reference GPS alt

    /* Code 1-byte string size. */
    std::string strVehicleID;     // Vehicle id, 128 bytes at most. Code 1-byte string size.

    /* sensor data version */
    float32_t sensorDataSpecVersion;

    RoadDatabaseHead_t(): headerLen(0),
                                      next(SNIPPET_PAYLOAD_TYPE_MAX_E),
                                      version(0),
                                      refTimeStamp(0LL),
                                      refGpsLon(0),
                                      refGpsLat(0),
                                      refGpsAlt(0),
                                      sensorDataSpecVersion(0)
    {
    }

    void toStringStream(std::stringstream &ss)
    {
        if (!ss.good())
        {
            return;
        }

        ss << "Header Info:"    << std::endl;
        ss << "  headerLen: "   << headerLen << std::endl;
        ss << "  payloadType: " << static_cast<uint32_t>(next) << std::endl;
        ss << "  version: "     << version << std::endl;
        ss << "  reference timeStamp: " << refTimeStamp << std::endl;
        ss << "  reference gps: " << refGpsLon << ","
                                  << refGpsLat << ","
                                  << refGpsAlt << std::endl;
    }
};

struct ModelConfigBase_t
{
    /* Camera parameters */
    float32_t   fx = 0.0F;
    float32_t   fy = 0.0F;
    float32_t   cx = 0.0F;
    float32_t   cy = 0.0F;

    /* Camera install information */
    float32_t   yaw = 0.0F;
    float32_t   pitch = 0.0F;
    float32_t   roll = 0.0F;
    float32_t   cameraHeight = 0.0F;

    /* Image size */
    uint32_t    length = 0;
    uint32_t    width = 0;

    SCALE_E     slamScale = SCALE_MAX_E;

    /* SLAM descriptor type of 2D points */
    SLAM_DESCRIPTOR_TYPE_E descriptorType = SLAM_DESCRIPTOR_TYPE_SGD_FLOAT32_256B_E;

    /* basis mat of descriptor*/
    float32_t   basisMat[48 * 24] = {0.0f};

    uint64_t    baseTimestamp = 0LL;
};

struct ModelConfigSlam_t : public ModelConfigBase_t
{
    float32_t   hoodHeight = 0.0F;
};

struct ModelConfigVehicle_t : public ModelConfigSlam_t
{
    SCALE_E     voxelScale = SCALE_MAX_E;
};

inline void copyModelConfig(const ModelConfigSlam_t &slamConfig, ModelConfigVehicle_t &vehicleConfig)
{
    /* Camera parameters */
    vehicleConfig.fx = slamConfig.fx;
    vehicleConfig.fy = slamConfig.fy;
    vehicleConfig.cx = slamConfig.cx;
    vehicleConfig.cy = slamConfig.cy;

    /* Camera install information */
    vehicleConfig.yaw = slamConfig.yaw;
    vehicleConfig.pitch = slamConfig.pitch;
    vehicleConfig.roll = slamConfig.roll;
    vehicleConfig.cameraHeight = slamConfig.cameraHeight;

    /* Image size */
    vehicleConfig.length = slamConfig.length;
    vehicleConfig.width = slamConfig.width;
    vehicleConfig.slamScale = slamConfig.slamScale;

    /* SLAM descriptor type of 2D points */
    vehicleConfig.descriptorType = slamConfig.descriptorType;
    vehicleConfig.baseTimestamp = slamConfig.baseTimestamp;
    vehicleConfig.hoodHeight = slamConfig.hoodHeight;

    memcpy(vehicleConfig.basisMat, slamConfig.basisMat, 48 * 24 * sizeof(float32_t));

    return;
}

struct ModelConfigServer_t : public ModelConfigVehicle_t
{
    std::string rtvName;
};

struct ObserveKfInfo_t
{
    // This structure shows the the object is seen
    int16_t    startKfIdx;      // start keyframe Index,  vector index of slam data's keyframe set
    int16_t    endKfIdx;        // end keyframe Index,  vector index of slam data's keyframe set

    ObserveKfInfo_t(): startKfIdx(0), endKfIdx(0) {}
    ObserveKfInfo_t(int16_t iStartKf, int16_t iEndKf): startKfIdx(iStartKf), endKfIdx(iEndKf) {}
};

struct PayloadBase_t
{
    SNIPPET_PAYLOAD_TYPE_E  type;                // payload type

    PayloadBase_t(SNIPPET_PAYLOAD_TYPE_E  payloadType = SNIPPET_PAYLOAD_TYPE_MAX_E): type(payloadType)
    {
    }

    virtual ~PayloadBase_t(){};
};

/***********************************************
 * Camera instrinic parameters.
 ***********************************************/
struct InstrinicParam_t
{
    InstrinicParam_t(float32_t fx_, float32_t fy_,
                               float32_t cx_, float32_t cy_,
                               float32_t k1_ = 0.f, float32_t k2_ = 0.f, float32_t k3_ = 0.f,
                               float32_t p1_ = 0.f, float32_t p2_ = 0.f)
    {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;

        k1 = k1_;
        k2 = k2_;
        k3 = k3_;
        p1 = p1_;
        p2 = p2_;
    }

    bool operator == (const InstrinicParam_t& param)
    {
        const float EPS = 2.0f;

        return (std::fabs(param.fx - fx) <= EPS &&
                std::fabs(param.fy - fy) <= EPS &&
                std::fabs(param.cx - cx) <= EPS &&
                std::fabs(param.cy - cy) <= EPS);
    }

    float32_t fx;
    float32_t fy;
    float32_t cx;
    float32_t cy;

    float32_t k1;
    float32_t p1;
    float32_t k2;
    float32_t p2;
    float32_t k3;
};

inline uint32_t getCodeSnippetVersion(SNIPPET_PAYLOAD_TYPE_E payloadType)
{
    uint32_t codeVersion = 0;

    switch (payloadType)
    {
    case SNIPPET_PAYLOAD_TYPE_SLAM_E:
        codeVersion = RDB_SLAM_SNIPPET_VERSION;
        break;

    case SNIPPET_PAYLOAD_TYPE_LM_E:
        codeVersion = RDB_LM_SNIPPET_VERSION;
        break;

    default:
        break;
    }

    return codeVersion;
}

}

#endif

