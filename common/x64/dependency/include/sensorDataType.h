/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   sensorDataType.h
 * @brief  sensor data type definition.
 *******************************************************************************
 */
#ifndef SENSOR_DATA_TYPE_H
#define SENSOR_DATA_TYPE_H

#include <string.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define GPS_OUT_RANGE -1 


namespace libSensor
{
class Point3d
{
   public:
    Point3d() : x(0), y(0), z(0) {}
    Point3d(double xin, double yin, double zin) : x(xin), y(yin), z(zin) {}
    double x;
    double y;
    double z;
};

struct IndexRange
{
    int64_t start;
    int64_t end;
};

enum SENSOR_TYPE_E
{
    SENSOR_TYPE_UNDEFINED_E = 0,
    SENSOR_TYPE_GPS_E,
    SENSOR_TYPE_9DOFIMU_E,
    SENSOR_TYPE_IMAGE_E,
    SENSOR_TYPE_RAWGPS_E,
    SENSOR_TYPE_OBD_E,
    SENSOR_TYPE_QUALITY_E,
    SENSOR_TYPE_CAN_E,
    SENSOR_TYPE_RAWIMG_E
};

/*
the parent class of all sensor's data
*/
class BaseSensorData
{
   public:
    SENSOR_TYPE_E type;
    unsigned int number;
    uint64_t sensorTimestamp;
    uint64_t arrivalTimestamp;

    BaseSensorData(SENSOR_TYPE_E sensorType = SENSOR_TYPE_UNDEFINED_E)
        : type(sensorType), number(0), sensorTimestamp(0), arrivalTimestamp(0)
    {
    }
    void reset();
    virtual ~BaseSensorData() {}
};

class GpsData : public BaseSensorData
{
   public:
    Point3d gps;//z for GPGGA <9> altitude, y for GPRMC <3><4> latitude, x for GPRMC <5><6> longtitude
    int usedSatNum;//GPGGA <7>
    double trackAngle;  // //GPRMC <8>, -1 for invalid

    // V2.0 data
    bool isV2Valid;// if the V2.0 data is valid
    double speed;//GPRMC <7>, -1 for invalid
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
    // V2.0 end
    GpsData(SENSOR_TYPE_E sensorType = SENSOR_TYPE_GPS_E)
        : BaseSensorData(sensorType),
          usedSatNum(-1),
          trackAngle(-1),
          isV2Valid(false),
          speed(-1),
          hdop(-1),
          pdop(-1),
          vdop(-1),
          magneticVariation(-1),
          gpsQualityIndicator(-1),
          ageOfDgps(-1),
          dgpsStationId(-1),
          selectionMode(-1),
          fixMode(-1),
          positionStatus(-1),
          magneticVariationDir(-1)
    {
    }
    void reset();
    virtual ~GpsData() {}
};

class Imu9DofData : public BaseSensorData
{
   public:
    Point3d acc;
    Point3d gyro;
    Point3d mag;

    Imu9DofData(SENSOR_TYPE_E sensorType = SENSOR_TYPE_9DOFIMU_E) : BaseSensorData(sensorType){}
    void reset();
    virtual ~Imu9DofData() {}
};

class OBDData : public BaseSensorData
{
   public:
    double speed = -1;

    OBDData(SENSOR_TYPE_E sensorType = SENSOR_TYPE_OBD_E) : BaseSensorData(sensorType){}
    void reset();
    virtual ~OBDData() {}
};


class CanData : public BaseSensorData
{
    public:
        char sDataType[56];
        double fValue = 0;


        CanData(SENSOR_TYPE_E sensorType = SENSOR_TYPE_CAN_E) : BaseSensorData(sensorType)
        {
            memset(sDataType,0,sizeof(sDataType));
        }
        void reset()
        {
            BaseSensorData::reset();
            memset(sDataType,0,sizeof(sDataType));
        }
        virtual ~CanData(){}
};



// reserve
#define MAX_RAW_GPS_FRAME_SIZE 2048

/*
the raw gps sensor's data
*/
class RawGpsData : public BaseSensorData
{
   public:
    unsigned int m_uSize = 0;
    unsigned char m_aRawGps[MAX_RAW_GPS_FRAME_SIZE+8];

    RawGpsData(SENSOR_TYPE_E sensorType = SENSOR_TYPE_RAWGPS_E) : BaseSensorData(sensorType)
    {
        memset(m_aRawGps,0,sizeof(m_aRawGps));
    }
};

inline bool splitString(const std::string& src, const char * delim, std::vector<std::string>& dst)
{
    std::size_t pos = 0;
    int32_t len = static_cast<int32_t>(strlen(delim));
    std::size_t begin = pos;
    pos = src.find(delim,begin);
    dst.clear();
    while(std::string::npos != pos)
    {
        dst.push_back(src.substr(begin,pos-begin));
        begin = pos + len;
        pos = src.find(delim,begin);
    }

    if (!src.empty())
    {
        dst.push_back(src.substr(begin));
    }
    return (0 < dst.size() ? true : false);
}

inline double NDS2Degree(int32_t nds) { return nds * 90.0 / (1024 * 1024 * 1024); }

inline int32_t Degree2NDS(double degree)
{
    int32_t nds = (int32_t)(degree * (1024 * 1024 * 1024) / 90);
    return nds;
}
inline std::string toHumanReadString(GpsData &vData)
{
    char pBuf[1024] = {0};
    snprintf(pBuf, 1024,
             "num: %u, type: %d, sensorTimestamp: %llu, arrivalTimestamp: %llu, gps.x: %0.4f, gps.y: %0.4f, gps.z: %0.4f, "
             "usedSatNum: %d, speed: %0.4f, hdop: %0.4f, pdop: %0.4f, vdop: %0.4f, trackAngle: %0.4f, "
             "magneticVariation: %0.4f, gpsQualityIndicator: %d, ageOfDgps: %d, dgpsStationId: %d, selectionMode: %d, "
             "fixMode: %d, positionStatus: %d, magneticVariationDir: %d\n",
             vData.number, vData.type, static_cast<unsigned long long>(vData.sensorTimestamp),
             static_cast<unsigned long long>(vData.arrivalTimestamp), vData.gps.x, vData.gps.y, vData.gps.z, vData.usedSatNum,
             vData.speed, vData.hdop, vData.pdop, vData.vdop, vData.trackAngle, vData.magneticVariation,
             vData.gpsQualityIndicator, vData.ageOfDgps, vData.dgpsStationId, vData.selectionMode, vData.fixMode,
             vData.positionStatus, vData.magneticVariationDir);

    return pBuf;
}
inline std::string toHumanReadString(Imu9DofData &vData)
{
    char pBuf[1024] = {0};
    snprintf(pBuf, 1024,
             "num: %u, type: %d, sensorTimestamp: %llu, arrivalTimestamp: %llu, acc: %0.4f, %0.4f, %0.4f, gyro: %0.4f, "
             "%0.4f, %0.4f, mag: %0.4f, %0.4f, %0.4f\n",
             vData.number, vData.type, static_cast<unsigned long long>(vData.sensorTimestamp),
             static_cast<unsigned long long>(vData.arrivalTimestamp), vData.acc.x, vData.acc.y, vData.acc.z, vData.gyro.x,
             vData.gyro.y, vData.gyro.z, vData.mag.x, vData.mag.y, vData.mag.z);

    return pBuf;
}

inline std::string toHumanReadString(OBDData &vData)
{
    char pBuf[1024] = {0};
    snprintf(pBuf, 1024,
             "num: %u, type: %d, sensorTimestamp: %llu, arrivalTimestamp: %llu, speed: %.2lf\n",
             vData.number, vData.type, static_cast<unsigned long long>(vData.sensorTimestamp),
             static_cast<unsigned long long>(vData.arrivalTimestamp), vData.speed);

    return pBuf;
}

template<class T>
void CopyWithoutNumber(T & vDest, const T & vSrc){
    vDest.type = vSrc.type;
    memcpy( &vDest.sensorTimestamp, &vSrc.sensorTimestamp, (int)((uint64_t)((&vSrc) + 1) - (uint64_t)(&vSrc.sensorTimestamp) )  );
}

}  // namespace libSensor

#endif
