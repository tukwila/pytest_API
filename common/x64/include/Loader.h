/*******************************************************************************
 *                      RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LoadMode.cpp
 * @brief  load mode handler
 *******************************************************************************
 */

#pragma once

#include <boost/noncopyable.hpp>
#include <boost/thread/thread.hpp>
#include <condition_variable>
#include <TSQueue.hpp>
#include "VehicleAPIError.h"
#include "DivisionData.h"

namespace RDBVehicleAPI
{

class BaseLoader {
public:

    BaseLoader(const vehicle_conf_t& conf);
   
    virtual ~BaseLoader();
   
    virtual bool start();
    
    // virtual bool stop();

    virtual uint32_t updatePosition(const divisionIDSeq_t& divisionIds, const uint8_t layer, const roadDBCore::Point3d_t& gpsPos, vehicleDataCallBack_t callback);

    virtual uint32_t updatePosition(const divisionIDSeq_t& divisionIds, const uint8_t layer, const roadDBCore::Point3d_t& gpsPos);
   
    virtual uint32_t getDivisionDetails(const divisionIDSeq_t& divisionIds, std::vector<spDivisionData_t>& divisions);

    virtual uint32_t getDivisionTrajectories(const divisionIDSeq_t& divisionIds,  std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions);

    virtual uint32_t getDivisionTrajectories(std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions) const ;

    virtual void clearBuffer();

    uint32_t getNextDivisionIDs(const nodeID_t nodeId,  divisionIDSet_t &divisionIDs);
    	
	uint32_t getPassedDivisionsBySegs(const segmentIDSet_t& segmentIds, divisionIDSet_t& passedDivisions);
   
    const segmentIDSet_t &getCachedSegmentIDs() const;

    vehicle_conf_t conf_;
    boost::shared_ptr<DivisionData> pDivisionData_;
    std::atomic<bool> bInitOK_;
    vehicleDataCallBack_t callback_;
    DATA_LOAD_STATUS_E eLoadStatus_;   

private:
   BaseLoader() = delete;
   BaseLoader(const BaseLoader &rhs) = delete;
   BaseLoader &operator=(const BaseLoader &rhs) = delete;
};

class RealTimeVehicleDataLoader : public BaseLoader {
public:

    RealTimeVehicleDataLoader(const vehicle_conf_t& conf);
  
    virtual ~RealTimeVehicleDataLoader();

    bool start() override;
    
    void clearBuffer() override;

    uint32_t getDivisionDetails(const divisionIDSeq_t& divisionIds, std::vector<spDivisionData_t>& divisions) override;

   /* @bref step1: load all divisions by divisionIds, and load the next divisions by layer.
    *              then call back the cached data pointer the first time
    *       step2: update the position, load segments， and call back the cached data pointer the second time
    *       note:1.this interface updates data asynchronously,when data is updated successfully, call back the data.
    *            2.this interface only for real time mode, if current load mode is whole load mode, return error code
    * @param layer: division load layer,for example:
    *        if layer=0, only load all divisions by divisionIds,
    *        if layer=1, load all divisions by divisionIds,and load the next divisions of all the divisions
    *        if layer=2, load all divisions by divisionIds,and load the next divisions and next next divisions of all the divisions
    * @param divisionIds: the divisionIds to update the divisions
    * @param position: the position to update the data
    * @param callback: the call back function
    * @return RIV_VEHICLE_SUCCESS if update the position and call back data successfully
    */
    uint32_t updatePosition(const divisionIDSeq_t& divisionIds, const uint8_t layer, const roadDBCore::Point3d_t &position, vehicleDataCallBack_t callback) override;;

    
   /* @bref  if give divisionIds and layer, update all divisions by divisionIds, and load the next divisions by layer.
    *        if give positiob, update segments by position.
    * @param divisionIds: the divisionIds to update the divisions
    * @param layer: division load layer,for example:
    *        if layer=0, only load all divisions by divisionIds,
    *        if layer=1, load all divisions by divisionIds,and load the next divisions of all the divisions
    *        if layer=2, load all divisions by divisionIds,and load the next divisions and next next divisions of all the divisions
    * @param position: the position to update the segments
    * @return RIV_VEHICLE_SUCCESS if update successfully
    */
    uint32_t updatePosition(const divisionIDSeq_t& divisionIds, const uint8_t layer, const roadDBCore::Point3d_t &position) override;

	void loadData();	
	
private:

    RealTimeVehicleDataLoader() = delete;
    RealTimeVehicleDataLoader(const RealTimeVehicleDataLoader &rhs) = delete;
    RealTimeVehicleDataLoader &operator=(const RealTimeVehicleDataLoader &rhs) = delete;

	bool doLoadData();

    bool realChangeSegment(const roadDBCore::Point3d_t& newGps);

    uint32_t waitInitDataCached(const int32_t schedPolicy, const int32_t priority, const std::vector<int32_t>& vCpus);

private:
    roadDBCore::Segment segmentObj;
    std::shared_ptr<boost::thread> pThread_;
    TSQueue<roadDBCore::Point3d_t> reqPositions_;
    segmentID_t lastSegmentId_;
    struct timeval basicTime_;
    uint32_t tolerantTime_;
    std::mutex mutex_;
    std::condition_variable condition_;
};

class WholeVehicleDataLoader : public BaseLoader {
public:
    WholeVehicleDataLoader(const vehicle_conf_t& conf);
   
    virtual ~WholeVehicleDataLoader();
   
    bool start() override;
   
    void clearBuffer() override;

    uint32_t getDivisionTrajectories(const divisionIDSeq_t& divisionIds,  std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions) override;

    uint32_t getDivisionTrajectories(std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions) const override;

private:

    WholeVehicleDataLoader() = delete;
    WholeVehicleDataLoader(const WholeVehicleDataLoader &rhs) = delete;
    WholeVehicleDataLoader &operator=(const WholeVehicleDataLoader &rhs) = delete;
};
}