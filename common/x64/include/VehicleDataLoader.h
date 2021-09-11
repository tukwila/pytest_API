/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    riv_vehicle_api.h
 * @hrief   Interfase of vehicle comn api for Localization
 * @history
 *******************************************************************************
 */

#pragma once

#include <boost/noncopyable.hpp>
#include "VehicleAPIError.h"
#include "LogWrapper/LogWrapper.h"
#include "Loader.h"

namespace RDBVehicleAPI
{
class VehicleDataLoader : public boost::noncopyable
{   
public:
    
   /* @bref the constructor of the vehicle data loader
    * @param conf config the vehicle data loader
    *             conf.dbPath Vehicle db path
    *             conf.schedPolicy Thread scheduling strategy
    *             conf.priority Thread priority
    *             conf.vCpus CPUs to bind to
    *             conf.initialPosition when we use realtime data load mode, we must give the initial position  
    *             conf.cacheMode data load mode, whole mode or realtime mode
    */
   VehicleDataLoader(const vehicle_conf_t& conf);

   /* @bref the destructor of the vehicle data loader
    */
   ~VehicleDataLoader();

   /* @bref start the vehicle data loader
    * @return true if start successfully, otherwise false
    */
    bool start();

   /* @bref stop the vehicle data loader
    * @return true if stop successfully, otherwise false
    */
    // bool stop();

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
    uint32_t updatePosition(const divisionIDSeq_t& divisionIds, const uint8_t layer, const roadDBCore::Point3d_t &position, vehicleDataCallBack_t callback);

   /* @bref  if give divisionIds and layer, update all divisions by divisionIds, and load the next divisions by layer.
    *        if give position, update segments by position. 
    *        note: only for realtime mode, if current load mode is whole load mode, return error code
    * @param divisionIds: the divisionIds to update the divisions
    * @param layer: division load layer,for example:
    *        if layer=0, only load all divisions by divisionIds,
    *        if layer=1, load all divisions by divisionIds,and load the next divisions of all the divisions
    *        if layer=2, load all divisions by divisionIds,and load the next divisions and next next divisions of all the divisions
    * @param position: the position to update the segments
    * @return RIV_VEHICLE_SUCCESS if update successfully
    */
    uint32_t updatePosition(const divisionIDSeq_t& divisionIds, const uint8_t layer, const roadDBCore::Point3d_t &position);

   /* @bref Get ids of all divisions that pass the segments
    * @param segmentIds the ids of the segments
    * @param divisionIds the ids of all divisions that pass the given segments
    * @return RIV_VEHICLE_SUCCESS if get ids of all divisions that pass the segments successfully
    */
    uint32_t getPassedDivisionIDs(const segmentIDSet_t& segmentIds, divisionIDSet_t &divisionIds);

   /* @bref Get divisions by given division ids, only for real time mode
    *       if current load mode is whole load mode, return error code
    * @param divisionIds the given division ids
    * @param divisions the divisions
    * @return RIV_VEHICLE_SUCCESS if get divisions by given division ids successfully
    */
    uint32_t getDivisionDetails(const divisionIDSeq_t& divisionIds, std::vector<spDivisionData_t>& divisions);

    /* @brief get trajectories by division ids, only for whole load mode
     *        if current load mode is real time mode, return error code
     * @param divisionIds the request division ids
     * @param divisions divition trajectories
     * @return RIV_VEHICLE_SUCCESS  successfully, otherwise  failed
     */
    uint32_t getDivisionTrajectories(const divisionIDSeq_t& divisionIds,  std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions);

    /* @brief get all division trajectories, only for whole load mode,
     *        if current load mode is real time mode, return error code
     * @param divisions all loaded division trajectories
     * @return RIV_VEHICLE_SUCCESS  successfully, otherwise  failed
     */
    uint32_t getDivisionTrajectories(std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions) const;

   /* @bref Get ids of current cached segments, used only in whole mode
    */
    const segmentIDSet_t& getCachedSegmentIDs() const;

   /* @brief get next division ids by node id
    * @param nodeId the node id 
    * @param divisionIDs next division ids
    * @param lvl loglevel
    * @return RIV_VEHICLE_SUCCESS  successfully, otherwise  failed
    */
    uint32_t getNextDivisionIDs(const nodeID_t nodeId, divisionIDSet_t& divisionIDs);

    /* @brief set log level
     * @param fileName log file name
     * @param moduleName module name 
     * @param lvl loglevel
     * it should be called only once
     */
    void initLog(const std::string& fileName, const std::string& moduleName, const LOG_LEVEL lvl);

 /**
    ****************************************************************************
    * @brief clearBuffer - This function is used to clean the buffer
    * can use it or not ,it means whole system' termination
    *  <1> Parameter Description:
    *  <2> Detailed Description:
    *
    ****************************************************************************
    */
    void clearBuffer();

private:
    VehicleDataLoader() = delete;
    VehicleDataLoader(const VehicleDataLoader &rhs) = delete;
    VehicleDataLoader &operator=(const VehicleDataLoader &rhs) = delete;

    vehicle_conf_t conf_;
    SP(BaseLoader) loader_;
 
};

}

