/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DBConnect.h
 * @brief  
 *******************************************************************************
 */

#pragma once

extern "C"
{
#include <sqlite3.h>
}

#include <map>
#include <set>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

namespace RDBVehicleAPI
{
#define DBSUCCESS   (0)
typedef std::map<std::string, std::string>	dbRow_t;
typedef std::vector<dbRow_t> 				dbRowSeq_t;


enum SYNC_MODE_E
{
    SYNC_MODE_OFF_E = 0,
    SYNC_MODE_NORMAL_E = 1,
    SYNC_MODE_FULL_E = 2
} ;

class DBConnect
{
public:
    DBConnect(const std::string& sDbPath,bool bCreateNotExit = false,int32_t timeOut = 20000);
    ~DBConnect();
private:
    DBConnect(const DBConnect& other) = delete;
    DBConnect& operator=(const DBConnect& other) = delete;
public:
    bool open();
    bool close();
    bool beginTX();
    bool commitTX();
    bool rollTX();
    bool execSql(const std::string& strSqlCmd);
    bool execSql(const std::string& strSqlCmd, dbRowSeq_t& rows);
	
private:
    bool checkValid();
    bool setSynchronousMode(SYNC_MODE_E mode);
    bool execSql4SelBlob(const std::string& sSqlCmd, dbRowSeq_t& rows);
	
private:
    std::string DBPath_;
    bool bCreateNotExit_ = false;
    int32_t timeOut_ = 20000;
    sqlite3* pDB_ = NULL;
    bool bOpen_ = false;
};
// }
}
