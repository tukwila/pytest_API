/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   sensorFormatStreamReader.h
 * @brief  sensor data format stream reader.
 *******************************************************************************
 */
#ifndef SENSOR_FORMAT_STREAM_READER_H
#define SENSOR_FORMAT_STREAM_READER_H

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "sensorDataType.h"
namespace libSensor
{
//template <class T>class SensorDataParser;

template <class T>
class SensorFormatStreamReader
{
   public:
    virtual bool open(const std::string path) = 0;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool readData(std::vector<T> &vSensorData, const IndexRange &range = {-1, -1}) = 0;
    virtual bool readData(std::vector<T> &vSensorData, uint64_t ulStartArrivalTime) = 0;
    virtual bool readData(T &sensorData) = 0;
    virtual bool readData(uint64_t index, T &sensorData) = 0;
    virtual uint64_t dataCount() = 0;
    virtual const std::string &getSensorID() = 0;
    virtual ~SensorFormatStreamReader(){};
};

class CanFormatStreamReader
{
    public :
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool readData(const std::string &subDataType,std::vector<CanData> &vSensorData, const IndexRange &range = {-1, -1}) = 0;
    virtual bool readData(const std::string &subDataType,CanData &sensorData) = 0;
    virtual bool readData(const std::string &subDataType,uint64_t index, CanData &sensorData) = 0;
    virtual bool readData(std::vector<CanData> &vSensorData, uint64_t ulStartArrivalTime) = 0;
    virtual uint64_t dataCount(const std::string &subDataType) = 0;
    virtual uint64_t dataCount() = 0;
    virtual const std::string &getSensorID() = 0;
    virtual ~CanFormatStreamReader(){};
};

// Factory function
template <typename T>
std::shared_ptr<SensorFormatStreamReader<T>> makeDataReader(const std::string& path);

std::shared_ptr<CanFormatStreamReader> makeCanDataReader(const std::string& path);

}  // namespace libSensor

#endif
