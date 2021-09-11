/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   InputDataTypedef.h
 * @brief  Definition of data structures for localization input
 *******************************************************************************
 */
#pragma once


#include <vector>
#include "typeDef.h"
#include "CommunicateDef/RdbV2SGeometry.h"


namespace algo
{
namespace vehicle
{

using roadDBCore::Point3d_t;
using roadDBCore::uint8_t;
using roadDBCore::uint32_t;
using roadDBCore::float64_t;

enum LOC_MODE_E: uint8_t
{
    /*
      Standard Localization working mode.
      Input image, IMU and GPS data are received from sensor or simuloatr.
    */
    LOC_MODE_STD_LOC_E,

    /*
      RTV Localization working mode.
      Input image, IMU and GPS data are parsed from RTV file,
      IMU file and GPS file.
    */
    LOC_MODE_RTV_LOC_E,

    /*
      Recorder working mode.
      Input image, IMU and GPS data are received from sensor or simuloatr.
      Recorded data is saved as RTV file, IMU file and GPS file.
    */
    LOC_MODE_STD_REC_E,

    /*
      Standard Localization and Recorder working mode.
      Input image, IMU and GPS data are received from sensor or simuloatr.
      Recorded data is saved as RTV file, IMU file and GPS file.
    */
    LOC_MODE_STD_LOC_REC_E,

    LOC_MODE_MAX_E
};

enum IMAGE_UNDISTORTION_MODE_E: uint8_t
{
    IMAGE_UNDISTORTION_INVALID_E,   // unKown undistortion flag
    IMAGE_UNDISTORTION_UNDO_E,      // need undistort image
    IMAGE_UNDISTORTION_DONE_E,      // no need undistort image
    IMAGE_UNDISTORTION_MAX_E
};

const float DEFAULT_TIMESTAMP = 0.0;

struct BaseRawData_t
{
    uint32_t number = 0;
    float64_t absTimestamp = DEFAULT_TIMESTAMP;
    float64_t absGlbTimestamp = DEFAULT_TIMESTAMP;
};

struct BaseImageData_t
{
    bool status = false;
    uint32_t number = 0;
    float64_t relTimestamp = DEFAULT_TIMESTAMP;
    float64_t absTimestamp = DEFAULT_TIMESTAMP;
    float64_t absGlbTimestamp = DEFAULT_TIMESTAMP;
};

struct RawImageData_t: public BaseImageData_t
{
    std::vector<uint8_t> vecCharImage;
    
    uint8_t * pRawImg;// the raw img buffer will be invalid after get 30 frames from sensor,
                            //the data saved at pRawImg when sensor have raw img, or encoded data at vecCharImage,
                            // and pRawImg is Null
    int32_t iRawFormart;// The fomart of raw img, for example, TJPF_BGR
    uint32_t uRawImgSize; // the raw img size in pRawImg
    uint32_t uWidth;      // width of raw img
    uint32_t uHeight;     // height of raw img
    IMAGE_UNDISTORTION_MODE_E imageUndistortion;

    explicit RawImageData_t(std::size_t reservedImageSize = 0)
    {
        if (reservedImageSize > 0)
        {
            vecCharImage.reserve(reservedImageSize);
        }
        pRawImg = NULL;
        iRawFormart = 0;
        uRawImgSize = 0;
        uWidth = 0;
        uHeight = 0;
        imageUndistortion = IMAGE_UNDISTORTION_UNDO_E;
    }
};

struct GpsData_t
{
    uint32_t number = 0;
    float64_t timestamp = DEFAULT_TIMESTAMP;
    float64_t absGlbTimestamp = DEFAULT_TIMESTAMP;
    Point3d_t gps;//z for GPGGA <9>, y for GPRMC <3><4>, x for GPRMC <5><6>
    Point3d_t speed;

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
    char selectionMode;//GPGSA <1>, 0 for M, 1 for other
    char fixMode;//GPGSA <2>, -1 for invalid
    char positionStatus;//GPRMC <2>, 0 for A, 1 for other
    char magneticVariationDir;//GPRMC <11>, 0 for E, 1 for other
};

struct BaseImuData_t
{
    uint32_t number = 0;
    float64_t relTimestamp = DEFAULT_TIMESTAMP;
    float64_t absTimestamp = DEFAULT_TIMESTAMP;
    Point3d_t value;
};

using ImuAccData_t = BaseImuData_t;
using ImuGyroData_t = BaseImuData_t;
using ImuOrienData_t = BaseImuData_t;

struct RawImuData_t
{
    float64_t absGlbTimestamp = DEFAULT_TIMESTAMP;
    ImuAccData_t acc;
    ImuGyroData_t gyro;
    ImuOrienData_t mag;
};

struct ObdData_t
{
    uint32_t number = 0;
    float64_t timestamp = DEFAULT_TIMESTAMP;
    float64_t absGlbTimestamp = DEFAULT_TIMESTAMP;
    float64_t speed = -1;
};

struct CanData_t
{
    int32_t frameID = 0;
    float64_t arrivalTimestamp = DEFAULT_TIMESTAMP;
    float64_t absGlbTimestamp = DEFAULT_TIMESTAMP;
    double fValue = 0;
    std::string sDataType; // such as 'VehicleSpeed', 'YawRate'...
};

const uint32_t RESERVED_SECONDS = 30;

const uint32_t IMAGE_NUM_PER_SECOND = 30;
const uint32_t GPS_NUM_PER_SECOND = 10; //current max gps hz is 10
const uint32_t IMU_NUM_PER_SECOND = 100;
const uint32_t OBD_NUM_PER_SECOND = 5;
const uint32_t CAN_NUM_PER_SECOND = 100;

const uint32_t IMAGE_BUFFER_SIZE = IMAGE_NUM_PER_SECOND * RESERVED_SECONDS;
const uint32_t GPS_BUFFER_SIZE = GPS_NUM_PER_SECOND * RESERVED_SECONDS;
const uint32_t IMU_BUFFER_SIZE = IMU_NUM_PER_SECOND * RESERVED_SECONDS;
const uint32_t OBD_BUFFER_SIZE = OBD_NUM_PER_SECOND * RESERVED_SECONDS;
const uint32_t CAN_BUFFER_SIZE = CAN_NUM_PER_SECOND * RESERVED_SECONDS;

const uint32_t PACKED_OBJECT_BUFFER_SIZE = IMAGE_NUM_PER_SECOND * 100;

} // namespace vehicle
}  // namespace algo

