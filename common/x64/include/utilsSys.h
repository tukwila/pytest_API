/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   utilsSys.h
 * @brief  utilsSys 
 *******************************************************************************
 */

#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include <pthread.h>
#include <algorithm>
#include <unordered_map>
#include <boost/lexical_cast.hpp>
#include "VehicleAPICommon.h"
namespace RDBVehicleAPI
{

 /*time log tool  */
uint64_t getTimeMs();
uint64_t getTimeNow();
std::string getStrTimeNow();
float64_t getDiffMs(const struct timeval& start, const struct timeval& end);
struct timeval getTimeofDay();
#define TIME_DIFF_START             struct timeval time_diff_start = getTimeofDay(); 
#define TIME_DIFF_END               struct timeval time_diff_end = getTimeofDay(); 
#define TIME_DIFF_USEAGE            LOG_TRACE <<"TIME_DIFF_USEAGE|" \
                                         << getDiffMs(time_diff_start,time_diff_end) \
                                         << " MS " \
                                         ; 
#define TIME_DIFF_USEAGE_PER(cnt)   LOG_TRACE <<"TIME_DIFF_USEAGE_PER|" \
                                         << (getDiffMs(time_diff_start,time_diff_end)/cnt) \
                                         << " MS " \
                                         ; 

//do the shell cmd
std::string callShell(const std::string& cmd);

std::string trimLeft(const std::string &sStr, const std::string &s);
std::string trimRight(const std::string &sStr, const std::string &s);

std::string replace(const std::string &sString, const std::string &sSrc, const std::string &sDest);
void sepStr(std::vector<std::string>& outVt,const std::string &sStr, const std::string &sSep, bool withEmpty = false);
std::vector<std::string> split(const std::string& sStr, const std::string& sSep);

//file stata enam
enum PATH_STATE_E
{
	PATH_STATE_NOTEXIST_E = 0,
	PATH_STATE_TYPE_UNKNOW_E = 1,
	PATH_STATE_TYPE_FILE_E = 2,
	PATH_STATE_TYPE_DIR_E = 3,
};
PATH_STATE_E getFileState(const std::string &sFullFileName);

void save2file(const std::string &sFullFileName, const std::string &sFileData);

//get files in dir by  "vehilce*.db or logic*.db"
void getDirFilesByPreAndSufFix(const std::string& sDir, const std::string& preFix,  
                        const std::string& surFix, std::vector<std::string>& filePaths, 
                        const std::vector<std::string>& filter = std::vector<std::string>(), bool bRecur = true); 

void getLatestFileVersion(const std::string& dbPath, const std::string& preFix, const std::string& sufFix, 
    const std::string& preTimeStamp, const std::string& sufTimeStamp, std::string& version);

template<typename MapT,typename K,typename V>
bool getMapEle(const MapT& _map,const K& _k,V& _v)
{
    typename MapT::const_iterator it = _map.find(_k);
    typename MapT::const_iterator itE = _map.end();
    if (it == itE)
    {
        return false;
    }
    else
    {
        _v = it->second;
        return true;
    }
}

/*get map elements whose keys in ks*/
template<typename MapT,typename VecOrSetT>
bool getMapEles(const MapT& _map, const VecOrSetT& ks, MapT& _outMap, bool bCheckItem = true)
{
    typename MapT::const_iterator itF = _map.begin();
    typename MapT::const_iterator itNF = _map.end();
    typename VecOrSetT::const_iterator it = ks.begin();
    typename VecOrSetT::const_iterator itE = ks.end();
    for (; it != itE; it++)
    {
        itF = _map.find(*it);
        if (itNF == itF)
        {
            if (true == bCheckItem)
            {
				  std::cout << "key not find|"<< *it << std::endl;
				  return false;
            }
        }
        else
        {
            _outMap.insert(std::make_pair(itF->first, itF->second));
        }
    }
    return true;
}


template<typename MapT>
void combineMap(const MapT& _src, MapT& _dest)
{
    typename MapT::const_iterator it = _src.begin();
    typename MapT::const_iterator itE = _src.end();

    for (; it != itE; it++)
    {
        _dest.insert(std::make_pair(it->first, it->second));
    }
}

template<typename SetT>
void combineSet(const SetT& _src, SetT& _dest)
{
    //Combine src to dest
    typename SetT::const_iterator it = _src.begin();
    typename SetT::const_iterator itE = _src.end();

    for (; it != itE; it++)
    {
        _dest.emplace(*it);
    }
}

template<typename K,typename V>
void mapmerge(const std::map<K,V>& src,std::map<K,V>& dest)
{
    std::map<K,V> tmp;
    std::merge(src.begin(),src.end(),dest.begin(),dest.end(),std::inserter(tmp, tmp.begin()));
    dest.swap(tmp);
}

template<typename MapT>
void mapmergeV2(const MapT& src,MapT& dest)
{
     //batch merge for map
    typename MapT::const_iterator it = src.begin();
    typename MapT::const_iterator itEnd = src.end();
    for( ; it != itEnd; it++ )
    {
        dest.insert(std::make_pair(it->first,it->second));
    }
}

template<typename K,typename V>
void mapMergeValue(const std::unordered_map<K,std::set<V>>& src,std::unordered_map<K,std::set<V>>& dest)
{
    typename std::unordered_map<K,std::set<V>>::const_iterator it = src.begin();
    typename std::unordered_map<K,std::set<V>>::const_iterator itEnd = src.end();
    for( ; it != itEnd; it++ )
    {
        dest[it->first].insert(it->second.begin(), it->second.end());
    }
}

template<typename MapT,typename SetT>
void mapBatchErase(MapT& kvs,const SetT& keys)
{
    //batch erase for map
    typename SetT::const_iterator it = keys.begin();
    typename SetT::const_iterator itE = keys.end();
    for(;it!=itE;it++)
    {
        kvs.erase(*it);
    }
}

/**
 *******************************************************************************
 * @brief typeCover
 *
 *  <1> Parameter Description:
 * <2> Detailed Description:
 *  Cover type from T1 to T2
*   return false when exception occurs
 *******************************************************************************
 */
template<typename T1,typename T2>
bool typeCover(const T1& src,T2& dest)
{
    try
    {
        dest = boost::lexical_cast<T2>(src);
        return true;
    }
    catch(std::exception& ex)
    {
        std::cout << ex.what() << "|" << src ;
        return false;
    }
}

template <typename T>
inline int32_t getFloor(T value)
{
    int32_t ret = (int)value;

    return ret - (ret > value);
}

template <typename T>
void vec2Set(const std::vector<T> inVec, std::set<T> &oSet)
{
    for(auto member:inVec)
    {
        oSet.insert(member);
    }

} 
}
