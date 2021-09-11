/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    DrawMessage.h
 * @brief   definition visualization display data structure.
 *
 * Change Log:
 *      Date             Who                      What
 *      2018.05.17       <qiang.li>               Created
 *******************************************************************************
 */
#ifndef DRAW_MESSAGE__H__
#define DRAW_MESSAGE__H__

#include <vector>
#include <iostream>
#include <memory>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Werror"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#endif
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include "typeDef.h"
#include "CommunicateDef/RdbV2SGeometry.h"
#include "algoInterface/Localization/InputDataTypedef.h"

namespace agent
{
//packet header
struct PackageHeader  
{  
    uint32_t uIndex;           //index
    uint32_t uTotalDataLen;    //total data size
    uint32_t uCurrentDataLen;  //current sent size  
    uint16_t uSplitCount;      //split count 
    uint16_t uCurrentIndex;    //current packet sub index
    uint32_t uCurrentOffset;   //current packet data offset 
    uint16_t uFinish;          //finish flag，0：finish，1：not finish
}; 	

const uint32_t MAX_LENGTH = 60 * 1024 + 128;
const uint32_t MAX_DATA_SEND_SIZE = 40 * 1024;

}

namespace roadDBCore
{

namespace rdbSerialization
{
using algo_vector3t = Eigen::Matrix<algo_float_t, 3, 1>;
using algo_matrix3t = Eigen::Matrix<algo_float_t, 3, 3>;
using algo_matrixXt = Eigen::Matrix<algo_float_t, Eigen::Dynamic, Eigen::Dynamic>;
using algo_vectorXt = Eigen::Matrix<algo_float_t, Eigen::Dynamic, 1>;

using namespace roadDBCore;

//message type.
enum DRAW_MSG_TYPE_E : uint8_t
{
    DRAW_MSG_NONE_TYPE_E = 0,
    DRAW_MSG_PREDICT_E = 1,  
    DRAW_MSG_UPDATE_E = 2,
    DRAW_MSG_GPS_E = 3,
    DRAW_MSG_SHOW_GPS_E = 4,
    DRAW_MSG_IMG_E = 5,
    DRAW_MSG_RPRJERR_E = 7,
    DRAW_MSG_STATUS_E = 8,
    DRAW_MSG_CAMINPARAM_E = 9,
    DRAW_MSG_IMU_PACK_E = 10,
    DRAW_MSG_DATA_RECORD_E = 11,
    DRAW_MSG_DIRECTION_E = 12,
    DRAW_MSG_DEBUG_CURVE_E = 13,
    DRAW_MSG_PURE_PREDICT_E = 14,
    DRAW_MSG_PMBA_E = 15,
    DRAW_MSG_OBD_PACK_E = 16,
    DRAW_MSG_LOG_INFO_E = 17,
    DRAW_MSG_SENSOR_STATUS_E = 18,
    DRAW_MSG_CUR_POS_E = 19,
    DRAW_MSG_COMPENSATE_E = 20,
    DRAW_MSG_FRAME_INFO_E = 30,
    DRAW_MSG_SENSOR_INFO_E = 31,
    DRAW_MSG_CAN_E = 32,
    DRAW_MSG_TYPE_MAX_E = 100
};

//base message 
struct MessageObject_t
{
    DRAW_MSG_TYPE_E eMsgType;
    
    MessageObject_t(DRAW_MSG_TYPE_E eTypeIn = DRAW_MSG_NONE_TYPE_E) : eMsgType(eTypeIn){}
};

//
struct MessagePredict_t : public MessageObject_t
{
    int32_t frameID;
    Point3d_t anchor;
    std::shared_ptr<algo_matrix3t> R_V2G;//，rotation 3 * 3
    std::shared_ptr<algo_vector3t> t_VinG; 
    std::shared_ptr<algo_vector3t> cov;//matrix 3*1

    MessagePredict_t() : MessageObject_t(DRAW_MSG_PREDICT_E), frameID(0) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//update message
struct MessageUpdate_t : public MessageObject_t
{
    int32_t frameID;
    int64_t taggedTs;            //image tagged: milliseconds

    Point3d_t anchor;
    std::shared_ptr<algo_matrix3t> R_V2G;//，rotation 3 * 3
    std::shared_ptr<algo_vector3t> t_VinG;
    std::shared_ptr<algo_vector3t> cov;//matrix 3*1

    //RT before correction
    std::shared_ptr<algo_matrix3t> R_G2C;//，rotation 3 * 3
    std::shared_ptr<algo_vector3t> t_GinC;
    
    //2d2d point 4 * m (x1,y1,x2,y2) inner point
    std::shared_ptr<algo_matrixXt> match2d2d; 
    // 2d3d point 5 * n (x1,y1,x2,y2,z2) inner and outer point
    std::shared_ptr<algo_matrixXt> match2d3d;
    //inner point 2d->3d
    std::shared_ptr<std::vector<size_t>> vec2d3dInlierIdx;
    std::shared_ptr<algo_vectorXt> pixelError;
    bool stopFlag = false;

    MessageUpdate_t() : MessageObject_t(DRAW_MSG_UPDATE_E), frameID(0) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MessageCompensate_t : public MessageObject_t
{
    int32_t frameID;
    int8_t  type;                   //-1:not avaiable 0:time 1:position 2:time + position, default:0
    int64_t reportTs;               //image report: milliseconds
 
    roadDBCore::Point3d_t anchor;
    
    std::shared_ptr<algo_matrix3t> R_G2C;//，rotation 3 * 3
    std::shared_ptr<algo_vector3t> t_GinC;
    std::shared_ptr<algo_vector3t> cov;//matrix 3*1

    MessageCompensate_t() : MessageObject_t(DRAW_MSG_COMPENSATE_E), frameID(0) {}

};

//Record GPS. 
struct MessageGps_t : public MessageObject_t
{
    int32_t frameID;
    roadDBCore::Point3d_t gps;
    roadDBCore::Point3d_t speed;
    float64_t timeStamp;
    int32_t channleID; //0 normal ; 1: DGPS

    int usedSatNum;//GPGGA <7>
    double trackAngle;  // //GPRMC <8>, -1 for invalid

    // V2.0 data
    bool isV2Valid;// if the V2.0 data is valid
    double oriSpeed;//GPRMC <7>, -1 for invalid
    double hdop;//GPGSA <16>, -1 for invalid
    double pdop;//GPGSA <15>, -1 for invalid
    double vdop;//GPGSA <17>, -1 for invalid
    double magneticVariation;//GPRMC <10>, -1 for invalid
    int gpsQualityIndicator;//GPGGA <6>, -1 for invalid
    int ageOfDgps;//GPGGA <13>, -1 for invalid
    int dgpsStationId;//GPGGA <14>, -1 for invalid
    char selectionMode;//GPGSA <1>, -1 for null, 0 for M, 1 for other
    char fixMode;//GPGSA <2>, -1 for invalid
    char positionStatus;//GPRMC <2>, -1 for null, 0 for A, 1 for other
    char magneticVariationDir;//GPRMC <11>, -1 for null, 0 for E, 1 for other

    MessageGps_t() : MessageObject_t(DRAW_MSG_GPS_E), frameID(0), timeStamp(0.0), channleID(0) {}
};

//Record img
struct MessageImg_t : public MessageObject_t
{
    int32_t frameID;
    bool imgStatus;
    float64_t relTimeStamp;
    float64_t arrivalTimestamp;
    int32_t channleID; 
    std::vector<uint8_t> vecCharImage;//IMG source frame（use in recive data）
    std::shared_ptr<cv::Mat> img;     //real IMG（used in visualization）
    algo::vehicle::IMAGE_UNDISTORTION_MODE_E imageUndistortion;

    MessageImg_t() : MessageObject_t(DRAW_MSG_IMG_E), frameID(0), 
                     imgStatus(false), relTimeStamp(0.0), 
                     arrivalTimestamp(0.0), channleID(0),
                     imageUndistortion(algo::vehicle::IMAGE_UNDISTORTION_MAX_E) 
                     {}
};

//Record can
struct MessageCan_t : public MessageObject_t
{
    int32_t frameID = 0;
    float64_t arrivalTimestamp = 0;
    double fValue = 0;
    std::string sDataType; // such as 'VehicleSpeed', 'YawRate'...

    MessageCan_t() : MessageObject_t(DRAW_MSG_CAN_E), frameID(0), 
                     arrivalTimestamp(0), fValue(0.0)
                     {}
};

//status message
enum LOC_EVENT_TYPE_E: uint8_t
{
    LOC_EVENT_TYPE_START,
    LOC_EVENT_TYPE_SEARCHED_MAP,
    LOC_EVENT_TYPE_INIT_SUCCESS,
    LOC_EVENT_TYPE_RESET,
    LOC_EVENT_TYPE_QUIT,
    LOC_EVENT_TYPE_MAX
};

struct MessageStatus_t : public MessageObject_t
{
    int32_t frameID;
    LOC_EVENT_TYPE_E event;
    std::string eventReason;    //like "reset reason", "quit reason";
    MessageStatus_t() : MessageObject_t(DRAW_MSG_STATUS_E) {}
};

struct MessageFrameInfo_t : public MessageObject_t
{
    int32_t frameID;
    std::vector<uint64_t> divisionIDs;   //current divisions
    MessageFrameInfo_t() : MessageObject_t(DRAW_MSG_FRAME_INFO_E) {}
};

//camara inner param.
struct MessageCamInParam_t : public MessageObject_t
{
    float32_t fx;
    float32_t fy;
    float32_t cx;
    float32_t cy;
    float32_t k1;
    float32_t k2;
    float32_t k3;
    float32_t p1;
    float32_t p2;

    float32_t scaleFactor;

    MessageCamInParam_t() : MessageObject_t(DRAW_MSG_CAMINPARAM_E),
                            fx(0.0), fy(0.0), cx(0.0), cy(0.0),
                            k1(0.0), k2(0.0), k3(0.0), p1(0.0), p2(0.0),scaleFactor(0.0) {}
};

struct ImuRecord_t
{
    int32_t frameID;
    roadDBCore::Point3d_t value;
    float64_t relTimeStamp;
    float64_t arrivalTimestamp;
    
    ImuRecord_t():frameID(0),relTimeStamp(0.0),arrivalTimestamp(0.0){}

    ImuRecord_t(int32_t id, roadDBCore::Point3d_t &val, float64_t relTime, float64_t absTime) 
    : frameID(id)
    , value(val)
    , relTimeStamp(relTime)
    , arrivalTimestamp(absTime)
    {}
};

//Record Imu
struct MessageImuPack_t : public MessageObject_t
{
    std::vector<ImuRecord_t> imuAcc;
    std::vector<ImuRecord_t> imuGyro;
    std::vector<ImuRecord_t> imuOri;

    MessageImuPack_t() : MessageObject_t(DRAW_MSG_IMU_PACK_E){}
};
    
//Record Imu
struct MessageSensorInfo_t : public MessageObject_t
{
    //sensor informations.
    int32_t iChannleID;//channle id
    std::string sSensorType;// imu,gps,img,obd,...
    std::string sInfoType;// "cameta_id",...
    std::vector<uint8_t> sInfoValue; // "cameta_id"->like this "0000000000000000000000000000000000004c63ebd00623"
 
    MessageSensorInfo_t () : MessageObject_t(DRAW_MSG_SENSOR_INFO_E), iChannleID(0){}
};
    
struct MessageObdPack_t : public MessageObject_t
{
    uint32_t number = 0;
    float64_t timestamp = 0.0;
    float64_t speed = -1;

    MessageObdPack_t() : MessageObject_t(DRAW_MSG_OBD_PACK_E){}
};


struct MessagePMBA_t : public MessageObject_t
{
    int32_t frameID;

    bool bDBLost;
    algo_vector3t ba;
    algo_vector3t bg;
    std::vector<algo_vector3t> triangulatedPointsPosition; //triangulated 3D points
    std::vector<cv::Point2d> trackedPoints;
    std::vector<cv::Point2d> triangulatedPoints;
    std::vector<cv::Point2d> validDbFeaturePoints;
    std::vector<cv::Point2d> invalidDbFeaturePoints;

    MessagePMBA_t() : MessageObject_t(DRAW_MSG_PMBA_E), frameID(0), bDBLost(true){}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum VIS_DEBUG_TYPE: uint8_t
{
    VIS_DEBUG_TYPE_MAJOR_INFO,
    VIS_DEBUG_TYPE_FATAL_ERROR,
    VIS_DEBUG_TYPE_MAX
};

const std::string VIS_DEBUG_MODULE_LOC =  "L"; //localization

struct MessageDebugInfo_t : public MessageObject_t
{
    VIS_DEBUG_TYPE type;
    std::string moduleName; //executable name: locAgent,rviz,localization,show_map
    std::string mainTitle;
    std::string subTitle; 
    
    MessageDebugInfo_t() : MessageObject_t(DRAW_MSG_LOG_INFO_E),
    moduleName(VIS_DEBUG_MODULE_LOC)//currently, all udp messages are from loc
    {}
};

struct MessageSensorStatus_t : public MessageObject_t
{
    //sensor miss count.
    int8_t imgMissCount; // if no such sensor, set to -1
    int8_t gpsMissCount;
    int8_t imuMissCount;
    int8_t obdMissCount;

    MessageSensorStatus_t() : MessageObject_t(DRAW_MSG_SENSOR_STATUS_E)
                            , imgMissCount(0), gpsMissCount(0)
                            , imuMissCount(0), obdMissCount(0)
                             {}
};


enum COMPENSATE_TYPE_E: int32_t
{
    COMPENSATE_TYPE_INVALID_E = -1,  // Compensate invalid
    COMPENSATE_TYPE_TIME_E = 0,      // Compensate with time, default
    COMPENSATE_TYPE_SPACE_E = 1,     // Compensate with space
    COMPENSATE_TYPE_TIMESPACE_E = 2, // Compensate both with time and space
    COMPENSATE_TYPE_MAX_E
};

struct MessageCurrentPos_t : public MessageObject_t
{
    int32_t frmIdx; //frame id

    int64_t taggedTs; // image tagged timestamp, ms
    float64_t nonCompensatedLongtitude;  //degree [-180~180]
    float64_t nonCompensatedLantitude;   //degree [-90~90]
    float64_t nonCompensatedAltitude;    //height ,meter
    float32_t nonCompensatedHeading;     // NED(North:0, East:90, Clockwise) [0~360]

    COMPENSATE_TYPE_E compensateType;
    int64_t reportTs;
    float64_t compensatedLongtitude;
    float64_t compensatedLantitude;
    float64_t compensatedAltitude;
    float32_t compensatedHeading;

    MessageCurrentPos_t() : MessageObject_t(DRAW_MSG_CUR_POS_E),
                            frmIdx(0),
                            taggedTs(0),
                            nonCompensatedLongtitude(0.0),
                            nonCompensatedLantitude(0.0),
                            nonCompensatedAltitude(0.0),
                            nonCompensatedHeading(0.0),
                            compensateType(COMPENSATE_TYPE_INVALID_E),
                            reportTs(0),
                            compensatedLongtitude(0.0),
                            compensatedLantitude(0.0),
                            compensatedAltitude(0.0),
                            compensatedHeading(0.0)
                            {}

};

}//! namespace rdbSerialization
}//! namespace RoadDBCore

#endif//! DRAW_MESSAGE__H__
