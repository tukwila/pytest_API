/*******************************************************************************
 *                      RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DivisionData.h
 * @brief  DivisionData
 *******************************************************************************
 */

#pragma once

#include <boost/noncopyable.hpp>
#include <mutex>
#include <unordered_map>
#include <atomic>
#include "VehicleTypes.h"
#include "VehicleAPICommon.h"
#include "VehicleAPIError.h"

namespace RDBVehicleAPI
{

class DbDirectory : public boost::noncopyable
{
public:
    DbDirectory(const std::string& dbDir);

    ~DbDirectory();

public:
    bool init();
    void destroy();
    
    // get vehicleDB path by segmentID
    bool getVehicleDbPaths(const std::string& segmentId, std::string& dbPath);

    bool getDbFilePaths();

    // get masterDB path
    std::string getMasterDbPath();

private:
    bool extractMasterDbPath();

protected:
    std::string masterDbPath_;
    std::vector <std::string> dbPaths_;
    std::string dbDir_;
};
typedef boost::shared_ptr<DbDirectory> DbDirectoryPtrT;

class DivisionSummay
{
public:

    DivisionSummay(const std::string& masterDbPath);
    ~DivisionSummay(){}

    // get passedDivisions by segments
    bool getPassedDivisionsBySegs(const segmentIDSet_t& segmentIds, divisionIDSet_t& passedDivisions);

    // get all divisionIDs
    void getAllSegAndDivisionIds(segmentIDSet_t& segIds, divisionIDSet_t& divisionIds);

    // get masterDB data from masterDB file
    bool loadMasterDB();

    void buildDivisionConnection(const divisionID_t divisionId, const nodeID_t startNodeId, const nodeID_t endNodeId);

    bool getNextDivisionIDs(const nodeID_t nodeId,  divisionIDSet_t &divisionIDs);

    void destroy();
private:
    std::string masterDbPath_;
    seg2DivisionSetMap_t seg2PassDivisionIdView_;
    // division2SegIdMap_t division2SegIdMap_;
    mapNodeIds_t mapNodes_;
    mapDivisionIds_t mapDivisions_;
};
typedef boost::shared_ptr<DivisionSummay> DivisionSummayPtrT;

enum DESERIALIZE_RLT_E
{
	DESERIALIZE_RLT_SUCCESS_E = 0,
	DESERIALIZE_RLT_FAIL_E,
	DESERIALIZE_RLT_FILE_NOT_EXIST_E

};

class DivisionData : public boost::noncopyable
{
public:
   
     DivisionData(const string &path);
   
    ~DivisionData(){}

public:
    
    uint32_t init(converter_t converter = nullptr);
   
    bool loadDataBySegmentIds(const segmentIDSeq_t& vecSegIds);
   
    void setReferencePoint(const roadDBCore::Point3d_t& gpsPos);
    // get passed divisions by segments
    bool getPassedDivisionsBySegs(const segmentIDSet_t& segmentIds, divisionIDSet_t& passedDivisions);

    //get division data by divisionIDs
    bool getDivisionDetails(const divisionIDSeq_t ids, std::vector<spDivisionData_t>& divisions);

    //get nodeId's next divisionIds
    bool getNextDivisionIDs(const nodeID_t nodeId,  divisionIDSet_t &divisionIDs);

    uint32_t getDivisionTrajectories(const divisionIDSeq_t& divisionIds,  std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions);

    uint32_t getDivisionTrajectories(std::vector<SP(const roadDBCore::VehicleDivision_t)>& divisions) const;

    uint32_t loadDivisions(const divisionIDSeq_t& divisionIds, const uint8_t layer);

    const segmentIDSet_t &getCachedSegmentIDs() const; 

    bool loadAllVehicleData(bool loadDivisionDetailInfo = false); 

    void destroy();
    
    void clearBuffer();

    const mapDivisionData_t &getCachedDivisions();

private:

    DivisionData() = delete;

    bool initSummary(const std::string& masterDbPath);
    
    bool initDirectory(const std::string& dbDir);

    void diffCacheDivIds(const segmentIDSet_t& segIds, divisionIDSet_t& divToAdd,  divisionIDSet_t& divToDel);

    DESERIALIZE_RLT_E deserializeVehicleDB(const divisionID_t id, divisionDataDetailPtr_t& division, bool loadDivisionDetailInfo = true);
   
    bool isDivisionLoaded(divisionID_t id);

private:
    DbDirectoryPtrT        pDbDirectory_;
    DivisionSummayPtrT     pSummary_;
    segmentIDSet_t          cacheSegIds_;
    roadDBCore::Point3d_t   refPoint_;
    std::mutex              dataMtx_;
    mapDivisionData_t       mapDivisions_;
    string                  path_;
    converter_t converter_;

    /*the cached data, for whole mode*/
    std::map<divisionID_t , SP(const roadDBCore::VehicleDivision_t)> vecDivisionTrajectories_;
};
}


