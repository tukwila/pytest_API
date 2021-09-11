/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   log_helper.h
 * @brief  helper for log struct
 *         one limit for use: must put a constant string before the new type
 *         for example LOG_INFO << "Landscape" << landscape;
 *******************************************************************************
 */
#pragma once
#include <unordered_set>
#include <unordered_map>
#include <iterator>
#include <vector>
#include <set>
#include <map>
#include "LogWrapper/LogWrapper.h"
#include "VehicleAPICommon.h"
#include "DBConnect.h"


class LogWrapper;
namespace RDBVehicleAPI
{  
template<typename T1>
LogWrapper &operator<<(LogWrapper& os, const std::vector<T1>& datas)
{
    std::ostringstream tmp;
    tmp << "vector[";
    for (std::size_t i = 0; i < datas.size(); i++)
    {
        tmp << datas[i];
        if(i != datas.size() - 1)
            tmp << ", ";
    }
    tmp << "]";

    os << tmp.str();

    return os;
}

template<typename T1>
LogWrapper &operator<<(LogWrapper& os, const std::set<T1>& datas)
{
    os << "set[";
    auto it = datas.begin();
    for(; it!= datas.end(); it++)
    {
        os << *it <<", ";
    }
    os << "]";
    return os;
}

template<typename T1, typename T2>
LogWrapper &operator<<(LogWrapper& os, const std::map<T1, T2>& datas)
{
    os << "map{";
    for (typename std::map<T1, T2>::const_iterator it = datas.begin();it != datas.end();++it)
    {
        os << "<"<<it->first << "," << it->second <<">";
    }
    os << "}";
    return os;
}

LogWrapper &operator<<(LogWrapper& os, const point3D_t& data);
LogWrapper &operator<<(LogWrapper& os, const WGS84_t& data);
LogWrapper &operator<<(LogWrapper& os, const NDSPoint_t& data);

// snapshot Road
LogWrapper &operator<<(LogWrapper& os, const dbRowSeq_t& data);

//LogWrapper &operator<<(LogWrapper &os, const laneConnect_t& data);


template<typename T1>
LogWrapper &operator<<(LogWrapper& os,const std::unordered_set<T1>& datas)
{
    os << "unordered_set[";
    auto it = datas.begin();
    for(; it!= datas.end(); it++)
    {
        os << *it <<", ";
    }
    os << "]";

    return os;
}

template<typename T1, typename T2>
LogWrapper &operator<<(LogWrapper& os, const std::unordered_map<T1, T2>& map)
{
    os << "map{";
    for (typename std::unordered_map<T1, T2>::const_iterator it = map.begin();it != map.end();++it)
    {
        os << "<"<<it->first << "," << it->second <<">";
    }
    os << "}";
    return os;
}

LogWrapper &operator<<(LogWrapper &os, const roadDBCore::Point3d_t& data);
LogWrapper &operator<<(LogWrapper &os, const roadDBCore::Point3f_t& data);
LogWrapper &operator<<(LogWrapper &os, const roadDBCore::Point2s_t& data);
LogWrapper &operator<<(LogWrapper &os, const roadDBCore::Point3d_t& data);

LogWrapper &operator<<(LogWrapper &os, const roadDBCore::VehicleNode_t& data);
LogWrapper &operator<<(LogWrapper &os, const roadDBCore::VehicleDivision_t& data);
//LogWrapper &operator<<(LogWrapper &os, const divisionData_t& data);

template<typename T1>
LogWrapper &operator<<(LogWrapper& os,const std::shared_ptr<T1>& p)
{
    if (p != nullptr)
    {
        os << *p;  
    }

    return os;
}

}
