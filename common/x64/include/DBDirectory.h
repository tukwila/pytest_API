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

namespace RDBVehicleAPI
{
class DBDirectory
{
public:
    DBDirectory();
    ~DBDirectory();

    DBDirectory(const DBDirectory& other) = delete;
    DBDirectory& operator=(const DBDirectory& other) = delete;
public:
    bool init(const std::string& dbDir, const EDBType dbtype);
    void destroy();

    // get vehicleDB path by segmentID
    bool getLogicDbPaths(const segmentID_t segmentId, std::string& dbPath) const;


    const segmentIDSet_t& getSegmentIds() const;
    // get masterDB path
    std::string getMasterDbPath();


private:
    const std::string getSegIdFromPath(const std::string& path) const;
    void getValidPbFileName(const std::string& dbPath, std::vector<std::string>& validFiles);
private:
    std::string masterDBPath_;
    std::map <segmentID_t,std::string> DBPaths_;
    segmentIDSet_t 		segmentIDs_;  // all segIds in realtime mode
    EDBType dbType_ = E_DB_TYPE_SQLITE;

};

// }
}
