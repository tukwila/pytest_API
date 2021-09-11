/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DBCommon.h
 * @brief  Definition of Error code and common struct.
 *******************************************************************************
 */

#ifndef  DB_COMMON_H_
#define  DB_COMMON_H_

#include <iostream>
#include <vector>
#include <set>
#include <string>
#include "utilityData.h"
#include "utilityFuns.h"
#include "CommunicateDef/RdbV2SGeometry.h"
#include "errorCode/serverSysErrorCode.h"

namespace roadDBCore
{

const uint32_t DB_VERSION_BACKEND = RDB_VERSION_MAIN + 5;
const uint32_t DB_VERSION_VEHICLE = RDB_VERSION_MAIN + 6;
const uint32_t DB_VERSION_VEHICLE_FILE = RDB_VERSION_MAIN + 11;
const uint32_t DB_VERSION_LOGIC = RDB_VERSION_MAIN + 4;
const uint32_t DB_VERSION_MAX = 0xffffffff;

enum DB_ERROR_E
{
    DB_SUCCESS = 0,
    DB_EXEC_SQL_ERROR = 1,
    DB_NO_RECORD = 2,
    DB_NO_SUCH_FIELD_NAME = 3,
    DB_OPEN_DB_ERROR = 4,
    DB_NOT_FOUND_SECTIONID = 5,
};

enum PROCESS_E: uint8_t
{
    PROCESS_SKELETON_LOADER_E = 0,
    PROCESS_REF_GENERATOR_E,
    PROCESS_REF_OPTIMIZER_E,
    PROCESS_REF_REGISTRATION_E,
    PROCESS_LOGIC_EXTRACTOR_E,
    PROCESS_STATUS_MAX_E
};

enum DATA_ACCESS_METHOD_E:uint8_t
{
    DA_METHOD_SQLITE_E,
    DA_METHOD_RESTFUL_E,
    DA_METHOD_MAX_E
};

enum ROW_ID_E
{
    DB_ROW_ID_INVALID = 0
};

template<uint32_t version>
struct VehicleDBData
{
    using HEAD_TYPE = void;
    using DATA_TYPE = void;
    static const bool bSupport = false;
};

template<uint32_t srcVersion, uint32_t dstVersion>
void convertVehicleDBData(const VehicleDBData<srcVersion> &srcData, VehicleDBData<dstVersion> &dstData)
{
    static_assert(srcVersion == dstVersion, "Data conversion is not supported!");
}

/*
convert char * into type T
*/
template<typename T> inline void string2RecordItem(const char * str, T& record)
{
    std::istringstream istr;

    istr.clear();
    istr.str(str);
    istr >> record;
}

/*
convert std::string into type T
*/
template<typename T> inline void string2RecordItem(const std::string &str, T& record)
{
    std::istringstream istr;

    istr.clear();
    istr.str(str);
    istr >> record;
}

/*
template function specification
convert char * into std::string
*/
template<> inline void string2RecordItem<std::string>(const char * str, std::string& record)
{
    record = str;
}

/*
template function specification
convert char * into std::string
*/
template<> inline void string2RecordItem<uint64_t>(const char * str, uint64_t& record)
{
    int64_t iTmp;
    std::istringstream istr;

    istr.clear();
    istr.str(str);
    istr >> iTmp;
    record = static_cast<uint64_t>(iTmp);
}

/*
convert type T into std::string
*/
template<typename T> inline void record2StringItem(const T& record, std::string& str)
{
    StringStreamX ostr;

    ostr.str("");
    ostr << record;
    str = ostr.str();
}

/*
template function specification
convert uint64 into std::string
*/
template<> inline void record2StringItem<uint64_t>(const uint64_t& record, std::string& str)
{
    StringStreamX ostr;
    int64_t iTmp = static_cast<int64_t>(record);;

    ostr.str("");
    ostr << iTmp;
    str = ostr.str();
}

/*
template function specification
convert uint8 into std::string
*/
template<>
inline void record2StringItem (const uint8_t& record, std::string& str)
{
    uint32_t iTmp = static_cast<uint32_t>(record);

    str = std::to_string(iTmp);
}

/*
template function specification
convert char into uint8
*/
template<>
inline void string2RecordItem<uint8_t>(const char * str, uint8_t& record)
{
    uint32_t iTmp;

    std::istringstream istr;
    istr.clear();
    istr.str(str);
    istr >> iTmp;
    record = static_cast<uint8_t>(iTmp);
}

/*
template function specification
convert Point3d_t into std::string
*/
template<>
inline void record2StringItem (const Point3d_t& pt, std::string& str)
{
    StringStreamX ostr;

    ostr.str("");
    ostr << pt.relLon << "," << pt.relLat << "," << pt.relAlt;
    str = ostr.str();
}

/*
template function specification
convert Point3d_t into std::string
*/
template<>
inline void record2StringItem (const Point3f_t& pt, std::string& str)
{
    StringStreamX ostr;

    ostr.str("");
    ostr << pt.relLon << "," << pt.relLat << "," << pt.relAlt;
    str = ostr.str();
}

/*
template function specification
convert char * into Point3d_t
*/
template<>
inline void string2RecordItem(const char * str, Point3d_t& record)
{
    StringStreamX ostr;

    ostr.str(str);

    char tmp;

    ostr >> record.relLon;
    ostr >> tmp;

    ostr >> record.relLat;
    ostr >> tmp;

    ostr >> record.relAlt;
}

//get the prefix of DB file from the stirng, including its path
int32_t  getDBFilePrefix(const std::string& dbFileName, std::string& dbFilePrefix);

SegmentID_t getSegID(const std::string& dbFileName);

uint64_t getDivID(const std::string& dbFileName);

//parser the segmentID list from the string
uint32_t  getSegIDList(const std::vector<std::string>& dbNameList, std::set<SegmentID_t>& segIDs);

//parser the segmentID list from the string
uint32_t  getSegIDList(const std::vector<std::string>& dbNameList, std::vector<SegmentID_t>& segIDs);


}

#endif
