/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    RIVLogicMgr.h
 * @hrief   Cpp interface of http://swagger.roaddb.ygomi.com/swagger-ui/?url=/yaml/master/road_database_viewer_api.yaml 
 * @history 
 *  1.If you need history of RIVLogicMgr befor c/s framework, pls refer to stash
 *    The latest release-tag of RIVLogicMgr befor c/s framework: release/20170914_2.0.5.x)
 *
 *  2.Fellow is change history of RIVLogicMgr base on c/s framework
 *    V1.0(20170915)
 *    1.This is the first version of RIVLogicMgr base on c/s framework
 *    2.Schema no change,business interface no change, just framework adjustment
 *
 *    V2.0(20170929)
 *    1.Only support V?(need confirm!!) db
 *    2.Schema changes: curve's poly3 ==> nurbs, other changes
 *  3.c/s frame is removed since 201807 for GM api
 *  4.New Horizon Model implemented
 *******************************************************************************
 */

#pragma once
#include "Conf.h"
#include "RIVErrCode.h"
#include "CHorizon.h"
#include "Geo.h"
#include "DataLoader.h"
#include "LogWrapper/LogWrapper.h"

// using namespace roaddb::geo;
namespace RIVAPI     = RDBVehicleAPI;
namespace RDBVehicleAPI
{
	    
const std::string logFileName = "xxrdb_api_log";
const std::string logModuleName = "rdb_api";

class RIVLogicMgr 
{
public:
    RIVLogicMgr();
    ~RIVLogicMgr();

public:
    friend class NDACache;

	/**
	 *******************************************************************************
	 * @brief init, logic api init method with config paras
	 *
	 *  <1> Parameter Description:
	 *       conf see definition of conf_t
	 *
	 *  @returns - RIV_COMN_SUCCESS if success, orthers if fail
	 *
	 *  <2> Detailed Description:
	 * all data will be loaded
	 *
	 *******************************************************************************
	 */
    RIVErrCode_t init(IN const conf_t& conf);
	
	/**
	 *******************************************************************************
	 * @brief init log 
	 *
	 *  <1> Parameter Description:
	 *
	 *  @param [In]  lvl log lvl
	 *
	 *  @return always RIV_COMN_SUCCESS
	 *
	 *  <2> Detailed Description:
	 *  init log Lvl， should be called only once 
	 *  
	 *******************************************************************************
	 */
    RIVErrCode_t initLog(IN const LOG_LEVEL lvl);
	/**
	 *******************************************************************************
	 * @brief destroy - common api stop
	 *
	 *  <1> Parameter Description:
	 *
	 *
	 *  @returns - always success
	 *
	 *  <2> Detailed Description:
	 * all data will be released
	 *
	 *******************************************************************************
	 */
    RIVErrCode_t destroy();
	/**
	 *******************************************************************************
	 * @brief updatePosition - update vehicle'scurrent Position
	 *
	 *  <1> Parameter Description:
	 *
	 *  @param [In]  position: current gps
	 *
	 *  @returns - RIV_COMN_SUCCESS if success, RIV_COMN_PARAS_ERROR if gps invalid
	 *			 RIV_COMN_CACHEAGENT_INIT_FAIL if cacheagent has been failed inited
	 *  <2> Detailed Description:
	 * used in realtime mode, when this function is called, new db data will be loaded
	 * according to the position with segment layers.
	 *******************************************************************************
	 */
    RIVErrCode_t updatePosition(IN const WGS84_t& position, updatePositionCallBack_t* cb = nullptr);
	RIVErrCode_t updatePosition_UtOnly(IN const WGS84_t& position, updatePositionCallBack_t* cb = nullptr);


	/**
	 *******************************************************************************
	 * @brief getHorizon - get  Horizon in a square whose center is position and 
	 *  half side length is range. 
	 *  
	 *  <1> Parameter Description:
	 *
	 *  @param [In]  position: center gps
	 *  			range: half side length of a square whose center is  position,
	 * 				in unit meter
	 *  @param [Out]  spHorizon
	 *                 output horizon pointer
	 *
	 *  @returns - RIV_COMN_SUCCESS if success, others fail
	 *        check if update positon has been  right called before this function, 
	 * 		  if yes, wait until RIV_COMN_SUCCESS return.
	 * 
	 *  <2> Detailed Description:
	 *  in realtime mode, it should be called some time after updatePosition, when 
	 *  all data are loaded
	 *  
	 *******************************************************************************
	 */
	RIVErrCode_t getHorizon(IN const WGS84_t& position, IN const double range, OUT SP_CONST(CHorizon)& spHorizon);
	
	/**
	 *******************************************************************************
	 * @brief getWholeHorizon - get Whole horizon
	 *
	 *  <1> Parameter Description:
	 *
	 *  @param [Out]  spHorizon
	 *                 output horizon pointer
	 *
	 *  @returns - RIV_COMN_SUCCESS if success, others fail
	 *
	 *  <2> Detailed Description:
	 *  get all the data of all db cached, 
	 *  use in whole mode in case of debug or demo, use getHorizon in other cases
	 *  
	 *******************************************************************************
	 */
    RIVErrCode_t getWholeHorizon(OUT SP_CONST(CHorizon)& spHorizon);

	/**
	 * @brief Get all divisionIDs with their segmentId, only use in whole
	 *
	 *  <1> Parameter Description: 
	 *
	 * 
	 * @param [Out] all divisionIDs with in map <segmentId,divisionIDs>
	 *
	 */
	RIVErrCode_t getAllDivisionIDs(OUT std::map<segmentID_t, std::set<divisionID_t>>& segDivisionsMap);

	/**
	 * @brief Get all data in the segments with given segment ids.
	 *
	 *  <1> Parameter Description: 
	 *
	 * @param [In] segmentIds the ids of the segments
	 * 
	 * @param [Out] spHorizon the data container containing all data in this segment
	 */
	RIVErrCode_t getDataBySegment(IN const std::vector<segmentID_t>& segmentIDs, OUT SP_CONST(CHorizon)& spHorizon);

	/**
	 *******************************************************************************
	 * @brief getPassRoadIDs - get segments' passedRoads 
	 *
	 *  <1> Parameter Description:
	 *
	 *  @param [In]  segmentIDs
	 *
	 *  @param [Out]  RoadIDs which pass segmentIDs
	 * 
	 *  @return always RIV_COMN_SUCCESS
	 *
	 *  <2> Detailed Description:
	 *  
	 *******************************************************************************
	 */
	RIVErrCode_t getPassRoadIDs(IN const std::vector<segmentID_t>& segmentIDs, OUT std::set<objectID_t>& passedRoadIDs);


	/**
	 * @brief Get all data by roadIDs 
	 *
	 *  <1> Parameter Description: 
	 *
	 * @param [In] roadIDs
	 * 
	 * @param [Out] spHorizon the data container containing all data in these roads
	 */
	
	RIVErrCode_t getDataByDivision(IN const std::set<divisionID_t>& divisionIDs, OUT SP_CONST(CHorizon)& spHorizon);
	
 	/**
	 * @brief Get the data version: corresponding vehicle data version and fixed data version
	 *		  only PB data supported
	 *  should be called after dataloaded, which means, after init in whole mode, 
	 *  and after updatePosition's call back return in realtime mode 
	 *  <1> Parameter Description:
	 *
	 * @param [Out] majorVersion: corresponding vehicle data version, it's the major version
	 *              fixVersion: data fixed version based on majorVersion
	 *  @return  RIV_COMN_UNSUPPORTED_CALL if source data is not PB
	 *           RIV_COMN_UNINITIALIZED if uninitialized
	 *           RIV_COMN_NO_DATA_LOADED if no data loaded
	 */         
	RIVErrCode_t getDataVersion(OUT std::string& majorVersion, OUT std::string& fixVersion);
private:
    bool checkRange(IN const float64_t range);
	SP_CONST(BaseRoadDataInfo) getRDRef() const ;
	SP_CONST(RoadDataSummary) getRDSummaryRef() const;
	const conf_t& getConf() const;


	void lock(const RWLOCKOP_E op);
	RIVErrCode_t doReport(const WGS84_t& gps, updatePositionCallBack_t* cb = nullptr);
	
	SP(BaseDataLoader) createDataLoader(const conf_t& conf);

private:
    volatile bool bInitOK_;
	SP(BaseDataLoader) pDataLoader_;
};
}

