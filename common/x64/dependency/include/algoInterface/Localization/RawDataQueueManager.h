/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RawDataQueueManager.h
 * @brief  Implementation of raw data queues manager
 *******************************************************************************
 */
#pragma once


#include <vector>
#include <memory>
#include "utility/SingletonManager.h"
#include "InputDataTypedef.h"
#include "ThreadUtils/LoopQueue.h"


namespace algo
{
namespace vehicle
{

enum RAW_DATA_E:uint8_t
{
    RAWDATA_IMAGE_E = 0,
    RAWDATA_GPS_E,
    RAWDATA_IMU_E,
    RAWDATA_OBD_E,
    RAWDATA_CAN_E,
    RAWDATA_MAX_E
};

/**
 *******************************************************************************
 * @class RawDataQueueManager
 *
 * @brief Manage the access of image queue, gps queue and IMU queue.
 *******************************************************************************
 */
class RawDataQueueManager
{
    using RawImageQueue_t = roadDBCore::LockFreeLoopQueue<std::shared_ptr<RawImageData_t>>;
    using RawGpsQueue_t = roadDBCore::LockFreeLoopQueue<std::shared_ptr<GpsData_t>>;
    using RawImuQueue_t = roadDBCore::LockFreeLoopQueue<std::shared_ptr<RawImuData_t>>;
    using RawObdQueue_t = roadDBCore::LockFreeLoopQueue<std::shared_ptr<ObdData_t>>;
    using RawCanQueue_t = roadDBCore::LockFreeLoopQueue<std::shared_ptr<CanData_t>>;

    using RawDataHandler_t = std::function<bool (const std::shared_ptr<BaseRawData_t> &)>;
    template<typename ModuleClass>
    using RawDataModuleHandler_t = bool (ModuleClass::*)(const std::shared_ptr<BaseRawData_t> &);
    using RawDataHandlerIter_t = std::multimap<uint32_t, RawDataHandler_t>::iterator;

public:
    template<typename CallableType>
    RawDataHandlerIter_t registerx(RAW_DATA_E id, CallableType handler)
    {
        std::unique_lock<std::mutex> ul(mutex_);

        return mmapHandlers_.emplace((uint32_t)id, (RawDataHandler_t)(handler));
    }

    template<typename ModuleClass>
    RawDataHandlerIter_t registerx(RAW_DATA_E id, ModuleClass &module, RawDataModuleHandler_t<ModuleClass> handler)
    {
        return registerx(id, std::bind(handler, &module, std::placeholders::_1));
    }

    bool execEvent(const RAW_DATA_E id, const std::shared_ptr<BaseRawData_t> &spRawData)
    {
        bool status = true;
        auto range =  mmapHandlers_.equal_range((uint32_t)id);

        for (auto iter = range.first; iter != range.second; ++iter)
        {
            auto handler = iter->second;
            status &= handler(spRawData);
            COM_LOG_TRACE << "Finish execution of raw Data: " << id;
        }

        if (!status)
        {
            COM_LOG_ERROR << "Failed to execute raw Data: " << id;
        }

        return status;
    }
    /**
     *******************************************************************************
     * @brief constructor
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - imageCapacity   The reserved image number in image queue.
     *
     *  @param [In]  - gpsCapacity   The reserved gps number in gps queue.
     *
     *  @param [In]  - imuCapacity   The reserved imu number in imu queue.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    explicit RawDataQueueManager(uint32_t imageCapacity = IMAGE_BUFFER_SIZE,
                                         uint32_t gpsCapacity = GPS_BUFFER_SIZE,
                                         uint32_t imuCapacity = IMU_BUFFER_SIZE,
                                         uint32_t obdCapacity = OBD_BUFFER_SIZE,
                                         uint32_t canCapacity = CAN_BUFFER_SIZE):
                                         imageQueue_(imageCapacity),
                                         gpsQueue_(gpsCapacity),
                                         imuQueue_(imuCapacity),
                                         imuForReportQueue_(imuCapacity),
                                         obdQueue_(obdCapacity),
                                         canQueue_(canCapacity)
    {
        imageQueueBlockValidItemThrd_ = 0;
        imageQueueBlockWaitMilliSeconds_= 0;
    }

    virtual ~RawDataQueueManager() {}

    /**
     *******************************************************************************
     * @brief write - Save an image into the image queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spObj   Share pointer to an image object
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool write(const std::shared_ptr<RawImageData_t> &spObj)
    {
        std::unique_lock<std::mutex> ul(mutex_);

        spObj->absGlbTimestamp = getCurrGlbTimestamp();
        preImageNumber_ = spObj->number;
        preImageTriggerAbsGlbTs_ = spObj->absGlbTimestamp;
        preImageTriggerTs_ = spObj->absTimestamp / 1000;

        while (imageQueueBlockWaitMilliSeconds_ && (imageSize() >= imageQueueBlockValidItemThrd_))
        {
            std::chrono::milliseconds waitTime(imageQueueBlockWaitMilliSeconds_);

            cond_blockWrite_.wait_for(ul, waitTime);
        }

        bool status = imageQueue_.write(spObj);

        if (!status)
        {
            LOG_ERROR << "write image failed, queue size:" <<  imageQueue_.size()
                      << ", image buffer size:" << IMAGE_BUFFER_SIZE;
        }

        cond_blockRead_.notify_one();

        std::shared_ptr<BaseRawData_t> spRawData = std::make_shared<BaseRawData_t>();
        spRawData->number = spObj->number;
        spRawData->absTimestamp = spObj->absTimestamp / 1000;
        spRawData->absGlbTimestamp = spObj->absGlbTimestamp;
        execEvent(RAWDATA_IMAGE_E, spRawData);
    
        return status;
    }

    /**
     *******************************************************************************
     * @brief write - Save an CAN data into the CAN queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spObj   Share pointer to an CAN object
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool write(const std::shared_ptr<CanData_t> &spObj)
    {
        spObj->absGlbTimestamp = getCurrGlbTimestamp();

        std::shared_ptr<BaseRawData_t> spRawData = std::make_shared<BaseRawData_t>();
        spRawData->number = spObj->frameID;
        spRawData->absTimestamp = spObj->arrivalTimestamp / 1000;
        spRawData->absGlbTimestamp = spObj->absGlbTimestamp;
        execEvent(RAWDATA_OBD_E, spRawData);

        return canQueue_.write(spObj);
    }

    /**
     *******************************************************************************
     * @brief write - Save a vector of CAN objects into CAN queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vSpObjects   CAN objects to be saved
     *
     *  @return The number of objects saved.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t write(const std::vector<std::shared_ptr<CanData_t>> &vSpObjects)
    {
        float64_t absGlbTimestamp = getCurrGlbTimestamp();
        size_t objectSize = vSpObjects.size();
        for (size_t i = 0; i < objectSize; i++)
        {
            vSpObjects[i]->absGlbTimestamp = absGlbTimestamp;
        }

        return canQueue_.write(vSpObjects);
    }

    void setImageQueueBlock(uint32_t validItemThrd, uint32_t waitMilliSeconds)
    {
        imageQueueBlockValidItemThrd_ = validItemThrd;
        imageQueueBlockWaitMilliSeconds_ = waitMilliSeconds;
    }

    void clearImageQueueBlock()
    {
        imageQueueBlockValidItemThrd_ = 0;
        imageQueueBlockWaitMilliSeconds_ = 0;
    }

    /**
     *******************************************************************************
     * @brief write - Save a GPS object into the gps queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spObj   Share pointer to a gps object
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool write(const std::shared_ptr<GpsData_t> &spObj)
    {
        spObj->absGlbTimestamp = getCurrGlbTimestamp();

        bool status = gpsQueue_.write(spObj);

        if (!status)
        {
            LOG_ERROR << "write gps failed, queue size:" << gpsQueue_.size()
                << ", gps buffer size:" << GPS_BUFFER_SIZE;
        }

        std::shared_ptr<BaseRawData_t> spRawData = std::make_shared<BaseRawData_t>();
        spRawData->number = spObj->number;
        spRawData->absTimestamp = spObj->timestamp / 1000;
        spRawData->absGlbTimestamp = spObj->absGlbTimestamp;
        execEvent(RAWDATA_GPS_E, spRawData);

        return status;
    }

    /**
     *******************************************************************************
     * @brief write - Save an IMU object into the IMU queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spObj   Share pointer to an IMU object
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool write(const std::shared_ptr<RawImuData_t> &spObj)
    {
        spObj->absGlbTimestamp = getCurrGlbTimestamp();
    
        bool status = imuQueue_.write(spObj);

        if (!status)
        {
            LOG_ERROR << "write imu failed, queue size:" << imuQueue_.size()
                      << ", imu buffer size:" << IMU_BUFFER_SIZE;
        }

        status = imuForReportQueue_.write(spObj);

        if (!status)
        {
            LOG_ERROR << "write imu failed, queue size:" << imuForReportQueue_.size()
                      << ", imu buffer size:" << IMU_BUFFER_SIZE;
        }

        std::shared_ptr<BaseRawData_t> spRawData = std::make_shared<BaseRawData_t>();
        spRawData->number = spObj->acc.number;
        spRawData->absTimestamp = spObj->acc.absTimestamp / 1000;
        spRawData->absGlbTimestamp = spObj->absGlbTimestamp;
        execEvent(RAWDATA_IMU_E, spRawData);

        return status;
    }

    /**
     *******************************************************************************
     * @brief write - Save a vector of image objects into image queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vSpObjects   Image objects to be saved
     *
     *  @return The number of objects saved.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t write(const std::vector<std::shared_ptr<RawImageData_t>> &vSpObjects)
    {
        std::unique_lock<std::mutex> ul(mutex_);

        float64_t absGlbTimestamp = getCurrGlbTimestamp();
        size_t objectSize = vSpObjects.size();
        for (size_t i = 0; i < objectSize; i++)
        {
            vSpObjects[i]->absGlbTimestamp = absGlbTimestamp;
            preImageNumber_ = vSpObjects[i]->number;
            preImageTriggerAbsGlbTs_ = absGlbTimestamp;
            preImageTriggerTs_ = vSpObjects[i]->absTimestamp / 1000;
        }

        while (imageQueueBlockWaitMilliSeconds_ && (imageSize() >= imageQueueBlockValidItemThrd_))
        {
            std::chrono::milliseconds waitTime(imageQueueBlockWaitMilliSeconds_);

            cond_blockWrite_.wait_for(ul, waitTime);
        }

        uint32_t num = imageQueue_.write(vSpObjects);

        cond_blockRead_.notify_one();

        return num;
    }

    /**
     *******************************************************************************
     * @brief write - Save a vector of gps objects into gps queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vSpObjects   GPS objects to be saved
     *
     *  @return The number of objects saved.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t write(const std::vector<std::shared_ptr<GpsData_t>> &vSpObjects)
    {
        float64_t absGlbTimestamp = getCurrGlbTimestamp();
        size_t objectSize = vSpObjects.size();
        for (size_t i = 0; i < objectSize; i++)
        {
            vSpObjects[i]->absGlbTimestamp = absGlbTimestamp;
        }
    
        return gpsQueue_.write(vSpObjects);
    }

    /**
     *******************************************************************************
     * @brief write - Save a vector of IMU objects into IMU queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vSpObjects   IMU objects to be saved
     *
     *  @return The number of objects saved.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t write(const std::vector<std::shared_ptr<RawImuData_t>> &vSpObjects)
    {
        float64_t absGlbTimestamp = getCurrGlbTimestamp();
        size_t objectSize = vSpObjects.size();
        for (size_t i = 0; i < objectSize; i++)
        {
            vSpObjects[i]->absGlbTimestamp = absGlbTimestamp;
        }
    
        imuForReportQueue_.write(vSpObjects);
        return imuQueue_.write(vSpObjects);
    }

    /**
     *******************************************************************************
     * @brief write - Save an OBD object into the OBD queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spObj   Share pointer to an OBD object
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool write(const std::shared_ptr<ObdData_t> &spObj)
    {
        spObj->absGlbTimestamp = getCurrGlbTimestamp();

        std::shared_ptr<BaseRawData_t> spRawData = std::make_shared<BaseRawData_t>();
        spRawData->number = spObj->number;
        spRawData->absTimestamp = spObj->timestamp / 1000;
        spRawData->absGlbTimestamp = spObj->absGlbTimestamp;
        execEvent(RAWDATA_OBD_E, spRawData);

        return obdQueue_.write(spObj);
    }

    /**
     *******************************************************************************
     * @brief write - Save a vector of OBD objects into OBD queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vSpObjects   OBD objects to be saved
     *
     *  @return The number of objects saved.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t write(const std::vector<std::shared_ptr<ObdData_t>> &vSpObjects)
    {
        float64_t absGlbTimestamp = getCurrGlbTimestamp();
        size_t objectSize = vSpObjects.size();
        for (size_t i = 0; i < objectSize; i++)
        {
            vSpObjects[i]->absGlbTimestamp = absGlbTimestamp;
        }

        return obdQueue_.write(vSpObjects);
    }

    /**
     *******************************************************************************
     * @brief read - Read an image object from image queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - spObj   Image object to be read
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(std::shared_ptr<RawImageData_t> &spObj)
    {
        bool status = imageQueue_.read(spObj);
        cond_blockWrite_.notify_one();
        return status;
    }

    /**
     *******************************************************************************
     * @brief read - Read an Image object from Image queue with specified maximum
     *               waiting time.
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - spObj   Image object to be read
     *
     *  @param [In]  - waitMilliSeconds   Maximum waiting time measured by millisecond
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(std::shared_ptr<RawImageData_t> &spObj, uint32_t waitMilliSeconds)
    {
        bool status = false;
        std::unique_lock<std::mutex> ul(mutex_);

        if (waitMilliSeconds)
        {
            std::chrono::milliseconds waitTime(waitMilliSeconds);

            cond_blockRead_.wait_for(ul, waitTime, [this] {return !isImageEmpty();});
        }

        if (!isImageEmpty())
        {
            imageQueue_.read(spObj);
            status = true;
            cond_blockWrite_.notify_one();
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief read - Read a GPS object from GPS queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - spObj   GPS object to be read
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(std::shared_ptr<GpsData_t> &spObj)
    {
        return gpsQueue_.read(spObj);
    }

    /**
     *******************************************************************************
     * @brief read - Read an IMU object from IMU queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - spObj   IMU object to be read
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(std::shared_ptr<RawImuData_t> &spObj, bool bReportImu = false)
    {
        if(bReportImu)
        {
            return imuForReportQueue_.read(spObj);
        }
        else
        {
            return imuQueue_.read(spObj);
        }
    }
    
    /**
     *******************************************************************************
     * @brief read - Read an OBD object from OBD queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - spObj   OBD object to be read
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(std::shared_ptr<ObdData_t> &spObj)
    {
        return obdQueue_.read(spObj);
    }

    /**
     *******************************************************************************
     * @brief read - Get all image objects available in image queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   image objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<RawImageData_t>> &vSpObjects)
    {
        return imageQueue_.read(vSpObjects);
    }

    /**
     *******************************************************************************
     * @brief read - Get all GPS objects available in GPS queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   GPS objects to be read
     *  @param [In]  - absTimestamp   the required gps time stamp
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<GpsData_t>> &vSpObjects, float64_t absTimestamp = 0.0D)
    {
        if (absTimestamp > DEFAULT_TIMESTAMP)
        {
            std::shared_ptr<GpsData_t> spGPS;

            while (gpsQueue_.peek(spGPS))
            {
                if (spGPS)
                {
                    if (spGPS->timestamp <= absTimestamp)
                    {
                        vSpObjects.push_back(spGPS);
                        gpsQueue_.pop();
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    gpsQueue_.pop();
                }
            }

            return true;
        }
        else
        {
            return gpsQueue_.read(vSpObjects);
        }
    }

    /**
     *******************************************************************************
     * @brief read - Get all OBD objects available in OBD queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   OBD objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<ObdData_t>> &vSpObjects)
    {
        return obdQueue_.read(vSpObjects);
    }

    /**
     *******************************************************************************
     * @brief read - Get all IMU objects available in IMU queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   IMU objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<RawImuData_t>> &vSpObjects, float64_t absTimestamp = 0.0D)
    {
        if (absTimestamp > DEFAULT_TIMESTAMP)
        {
            std::shared_ptr<RawImuData_t> spImu;

            while (imuQueue_.peek(spImu))
            {
                if (spImu)
                {
                    if (spImu->acc.absTimestamp <= absTimestamp)
                    {
                        vSpObjects.push_back(spImu);
                        imuQueue_.pop();
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    imuQueue_.pop();
                }
            }

            return true;
        }
        else
        {
            return imuQueue_.read(vSpObjects);
        }
    }

    uint32_t readReportImu(std::vector<std::shared_ptr<RawImuData_t>> &vSpObjects)
    {
        return imuForReportQueue_.read(vSpObjects);
    }
 
    /**
     *******************************************************************************
     * @brief read - Get all obd objects available in obd queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   obd objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<ObdData_t>> &vSpObjects, float64_t absTimestamp = 0.0D)
    {
        if (absTimestamp > 0.1)
        {
            std::shared_ptr<ObdData_t> spObd;

            while (obdQueue_.peek(spObd))
            {
                if (spObd)
                {
                    if (spObd->timestamp <= absTimestamp)
                    {
                        vSpObjects.push_back(spObd);
                        obdQueue_.pop();
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    obdQueue_.pop();
                }
            }

            return true;
        }
        else
        {
            return obdQueue_.read(vSpObjects);
        }
    }

    
    /**
     *******************************************************************************
     * @brief read - Read an CAN object from CAN queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - spObj   CAN object to be read
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(std::shared_ptr<CanData_t> &spObj)
    {
        return canQueue_.read(spObj);
    }

    /**
     *******************************************************************************
     * @brief read - Get all CAN objects available in CAN queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   CAN objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<CanData_t>> &vSpObjects)
    {
        return canQueue_.read(vSpObjects);
    }

    /**
     *******************************************************************************
     * @brief read - Get all CAN objects available in CAN queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vSpObjects   CAN objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    uint32_t read(std::vector<std::shared_ptr<CanData_t>> &vSpObjects, float64_t absTimestamp = 0.0D)
    {
        if (absTimestamp > 0.1)
        {
            std::shared_ptr<CanData_t> spObj;

            while (canQueue_.peek(spObj))
            {
                if (spObj)
                {
                    if (spObj->arrivalTimestamp <= absTimestamp)
                    {
                        vSpObjects.push_back(spObj);
                        canQueue_.pop();
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    canQueue_.pop();
                }
            }

            return true;
        }
        else
        {
            return canQueue_.read(vSpObjects);
        }
    }

    /**
     *******************************************************************************
     * @brief clear - Empty all internal queues.
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void clear()
    {
        imageQueue_.clear();
        gpsQueue_.clear();
        imuQueue_.clear();
        imuForReportQueue_.clear();
        obdQueue_.clear();
        canQueue_.clear();
    }

    /**
     *******************************************************************************
     * @brief isImageEmpty - Test whether the image queue is empty
     *
     *  <1> Parameter Description:
     *
     *  @return True if empty, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool isImageEmpty() const
    {
        return imageQueue_.empty();
    }

    /**
     *******************************************************************************
     * @brief isGpsEmpty - Test whether the GPS queue is empty
     *
     *  <1> Parameter Description:
     *
     *  @return True if empty, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool isGpsEmpty() const
    {
        return gpsQueue_.empty();
    }

    /**
     *******************************************************************************
     * @brief isImuEmpty - Test whether the IMU queue is empty
     *
     *  <1> Parameter Description:
     *
     *  @return True if empty, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool isImuEmpty(bool bReportImu = false) const
    {
        if (bReportImu)
        {
            return imuForReportQueue_.empty();
        }
        else
        {
            return imuQueue_.empty();
        }
    }

    /**
     *******************************************************************************
     * @brief isObdEmpty - Test whether the OBD queue is empty
     *
     *  <1> Parameter Description:
     *
     *  @return True if empty, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool isObdEmpty() const
    {
        return obdQueue_.empty();
    }

    /**
     *******************************************************************************
     * @brief isCanEmpty - Test whether the CAN queue is empty
     *
     *  <1> Parameter Description:
     *
     *  @return True if empty, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool isCanEmpty() const
    {
        return canQueue_.empty();
    }

    /**
     *******************************************************************************
     * @brief imageSize - Get the size of the image queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of images in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t imageSize() const
    {
        return imageQueue_.size();
    }

    /**
     *******************************************************************************
     * @brief gpsSize - Get the size of the GPS queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of GPS in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t gpsSize() const
    {
        return gpsQueue_.size();
    }

    /**
     *******************************************************************************
     * @brief imuSize - Get the size of the IMU queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of IMU in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t imuSize(bool bReportImu = false) const
    {
        if (bReportImu)
        {
            return imuForReportQueue_.size();
        }
        else
        {
            return imuQueue_.size();
        }
    }

    /**
     *******************************************************************************
     * @brief obdSize - Get the size of the OBD queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of OBD in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t obdSize() const
    {
        return obdQueue_.size();
    }

    /**
     *******************************************************************************
     * @brief canSize - Get the size of the CAN queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of CAN in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t canSize() const
    {
        return canQueue_.size();
    }

    uint32_t getCurImageNumber()
    {
        return preImageNumber_;
    }

    float64_t getCurImageGlbTs()
    {
        return preImageTriggerAbsGlbTs_;
    }

    float64_t getCurImageTs()
    {
        return preImageTriggerTs_;
    }
    
    /**
     * No copy constructor.
     */
    RawDataQueueManager(const RawDataQueueManager&) = delete;

    /**
     * No assignment.
     */
    RawDataQueueManager &operator=(const RawDataQueueManager&) = delete;

private:
    float64_t getCurrGlbTimestamp()
    {
        auto triggerTs = std::chrono::high_resolution_clock::now();
        auto triggerTsMs = std::chrono::duration_cast<std::chrono::milliseconds>(triggerTs.time_since_epoch()).count();
        return static_cast<float64_t> (triggerTsMs);
    }

    RawImageQueue_t imageQueue_;
    RawGpsQueue_t gpsQueue_;
    RawImuQueue_t imuQueue_;
    RawImuQueue_t imuForReportQueue_;
    RawObdQueue_t obdQueue_;
    RawCanQueue_t canQueue_;

    mutable std::mutex mutex_;
    std::condition_variable cond_blockRead_;
    std::condition_variable cond_blockWrite_;

    uint32_t imageQueueBlockWaitMilliSeconds_;
    uint32_t imageQueueBlockValidItemThrd_;

    std::multimap<uint32_t, RawDataHandler_t> mmapHandlers_;

    uint32_t preImageNumber_ = 0;
    float64_t preImageTriggerAbsGlbTs_ = 0.0;
    float64_t preImageTriggerTs_ = 0.0;
};

using RawDataQueue_t = RawDataQueueManager;

} // namespace vehicle
}  // namespace algo

