/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   sensorFormatStreamWriter.h
 * @brief  sensor data format stream writer.
 *******************************************************************************
 */
#ifndef SENSOR_FORMAT_STREAM_WRITER_H
#define SENSOR_FORMAT_STREAM_WRITER_H

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "sensorDataType.h"

namespace libSensor
{
template <class T>
class SensorFormatStreamWriter
{
   public:
    virtual bool open(const std::string path) = 0;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual void setSensorID(const std::string sensorID) = 0;
    virtual bool writeData(const std::vector<T>& vSensorData) = 0;
    virtual bool writeData(const T& sensorData) = 0;
    virtual ~SensorFormatStreamWriter(){};
};


class CanFormatStreamWriter
{
   public:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual void setSensorID(const std::string sensorID) = 0;
    virtual bool writeData(const std::vector<CanData>& vSensorData) = 0;
    virtual bool writeData(const CanData& sensorData) = 0;
    virtual ~CanFormatStreamWriter(){};
};




template <typename T>
std::shared_ptr<SensorFormatStreamWriter<T>> makeDataWriter(const std::string& path);

template <typename T>
std::string getFormatOutput(const T& sensorData, const std::string& header);


std::shared_ptr<CanFormatStreamWriter> makeDataWriter(const std::string& path);

std::string getFormatOutput(const CanData& sensorData, const std::string& header);

}  // namespace libSensor

#endif
