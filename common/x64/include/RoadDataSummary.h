/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RoadDataSummary.h
 * @brief  RoadDataSummary 
 *******************************************************************************
 */
#pragma once

#include "InnerStructs.h"
#include "DBConnect.h"
#include "Conf.h"
#include "VehicleAPICommon.h"

namespace RDBVehicleAPI
{
//RoadDataSummary
class RoadDataSummary 
{
public:
    RoadDataSummary();
    ~RoadDataSummary();

    RoadDataSummary(const RoadDataSummary& other) = delete;
    RoadDataSummary& operator=(const RoadDataSummary& other) = delete;
public:
    bool init(const std::string& dbPath, const EDBType dbtype);

    void destroy();

    bool getPassedRoadIdsbySegId(const segmentID_t segId, objectIDSet_t& roadIds) const;
    bool getRoadIDbyDivisionId(const divisionID_t divisionId, objectIDSet_t& roadIds) const;
    void getDivisionIDsByRoadIDs(const objectIDSet_t roadIDs, divisionIDSet_t &divisionIDs) const;
        
    const std::unordered_map<segmentID_t, objectIDSet_t>& getAllRoadIds() const;
private:
    //bool init();
    bool loadMasterData(const std::string& path);
    bool loadMasterDataPB(const std::string& path);
    
//     bool loadOneDbData();
// //    bool VerifyDBVersion(const SP(DBConnect) pDbConnect);
//     bool verifyTableLane(const SP(DBConnect)& pDbConnect);
//     bool verifyTableLaneConn(const SP(DBConnect)& pDbConnect);
//     bool doLoadVersion(const SP(DBConnect)& pDbConnect);
private:
    std::unordered_map<segmentID_t, objectIDSet_t>       seg2PassRoadIDs_;
    std::unordered_map<segmentID_t, objectIDSet_t>       seg2OwnedRoadIDs_;  // seg and roadIDs owned to seg
    std::unordered_map<divisionID_t, objectIDSet_t> division2RoadIDs_;
    std::unordered_map<objectID_t, divisionIDSet_t> road2DivisionIdSet_;
    std::string 				version_;

               
};

// }
}