/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   utilityFuns.h
 * @brief  utility functions
 *******************************************************************************
 */

#ifndef _OTHER_FUNC_H_
#define _OTHER_FUNC_H_

#include <execinfo.h>   // enbale backtrace()
#include <signal.h>
#include <vector>
#include <type_traits> // is_floating_point
#include <cmath>       // sin, cos
#include <string.h>
#include <set>
#include <limits.h>
#include "typeDef.h"
#include "utilityData.h"
#include "CommunicateDef/RdbV2SGeometry.h"


namespace roadDBCore
{


using std::vector;
using std::string;

struct DBDivisionDetail_t;
struct DBDivision_t;

const int32_t MAX_DISTANCE = INT_MAX;
const std::string TIMESTAMP_FORMAT = "%4u-%02u-%02u-%02u-%02u-%02u";


/**
 *******************************************************************************
 * @brief stringToNum - switch string to number
 *
 *  <1> Parameter Description:
 *
 *  @param [In] str - source string
 *
 *  @return num - dist number
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <class T>
inline T stringToNum(IN const std::string& str)
{
    std::istringstream iss(str);
    T num;

    iss >> num;

    return num;
}

/**
 *******************************************************************************
 * @brief numToString - switch number to string
 *
 *  <1> Parameter Description:
 *
 *  @param [In] num - source number
 *
 *  @param [Out] str - dist string
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <class T>
inline void numToString(IN T num, OUT std::string& str)
{
    str = std::to_string(num);

    return ;
}

/**
 *******************************************************************************
 * @brief isNumbers - check whether the input string is a number
 *
 *  <1> Parameter Description:
 *
 *  @param [In] str - the string of number
 *
 *  @return true or false
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
bool isNumbers(const std::string &str);

std::string getTimestampStr(const std::string &format = TIMESTAMP_FORMAT);

int32_t checkExist(const std::string& path);

enum FILE_DIR_TYPE_E
{
    IS_FILE = 0,
    IS_DIR
};

FILE_DIR_TYPE_E checkFileOrDir(const std::string& path);

bool checkFile(const std::string &file);

/**
********************************************************************************
 * @brief configLog - config log.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - const std::string &logFile
 *         output log filename.
 *
 *  <2> Detailed Description:
 *  config log. set log filename, level, type, color.
 *
 *  \ingroup schedule
 *******************************************************************************
 */
void configLog(const std::string &logFile);


/**
********************************************************************************
 * @brief getInputParam - get input value from main parameter.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] - value            store value.
 *
 *  @param [In] - paramValue       input parameter.
 *
 *  @param [In] - paramName        parameter value name.
 *
 *
 *  <2> Detailed Description:
 *  get input value from main parameter. if the parameter is null or length is 0
 *  return error and print the illegal parameter name.
 *
 *  \ingroup schedule
 *******************************************************************************
 */

int32_t getInputParam(std::string& value,
                      const char *paramValue,
                      const char *paramName);


/**
********************************************************************************
 * @brief getOptionalInputParam - get the optional input value from main parameter.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] - value            store value.
 *
 *  @param [In] - paramValue       input parameter, which may be NULL or blank
 *
 *  @param [In] - paramName        parameter value name.
 *
 *
 *  <2> Detailed Description:
 *  get the optional input value from main parameter. if the parameter is null or length is 0
 *  skip the parameter and return 0.
 *
 *  \ingroup schedule
 *******************************************************************************
 */
int32_t getOptionalInputParam(std::string& value,
                      const char *paramValue,
                      const char *paramName);

/**
********************************************************************************
 * @brief checkInputNullOpt - check input parameter is null.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - value         check paramName.
 *
 *  @param [In]  - paramName     check value name.
 *
 *
 *  <2> Detailed Description:
 *  check input parameters of input, if the parameter is null or length is 0 re-
 *  turn error and print the illegal parameter name.
 *
 *  \ingroup schedule
 *******************************************************************************
 */

int32_t checkInputNullOpt(const std::string& value,
                          const char *optName);

int32_t checkInputOpt(const std::string& value,
                          const char *optName,
                          FILE_DIR_TYPE_E type);


/**
 *******************************************************************************
 * @brief rdbRound- get round value of float
 *
 *  <1> Parameter Description:
 *
 *  @param [In] t - value
 *
 *  @return integral part of float
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <typename T>
RDB_INLINE int32_t rdbRound(T value)
{
     if (value > 0)
     {
         return static_cast<int32_t>(value + 0.5f);
     }
     else
     {
         return static_cast<int32_t>(value - 0.5f);
     }
}

/**
 *******************************************************************************
 * @brief rdbFloor- get round value of float
 *
 *  <1> Parameter Description:
 *
 *  @param [In] t - value
 *
 *  @return integral part of float
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <typename T>
RDB_INLINE int32_t rdbFloor(T value)
{
    int32_t ret = (int)value;

    return ret - (ret > value);
}

/**
 *******************************************************************************
 * @brief split-  split string into vector
 *
 *  <1> Parameter Description:
 *
 *  @param [In] src, source string
 *
 *  @param [In] separate_character, splite source string by this string
 *
 *  @return  vector, split result.
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
vector<string> split(const string& src, string separate_character);

/**
 *******************************************************************************
 * @brief computeHausDistance-  get Hausdorff distance of two data sets
 *
 *  <1> Parameter Description:
 *
 *  @param [In] Apoints, A data set
 *
 *  @param [In] Bpoints, B data set
 *
 *  @param [In] numA, size of A data set
 *
 *  @param [In] numB, size of B data set
 *
 *  @param [In] disType, type of Hausdorff distance
 *
 *  @return  distance.
 *
 *  <2> Detailed Description:
 *  disType = 1, use maximum distance; disType = 0, use mean distance;
 *  \ingroup
 *******************************************************************************
 */
double computeHausDistance(const std::vector<Point3d_t>& Apoints,
                           const std::vector<Point3d_t>& Bpoints,
                           int numA, int numB, int disType);

/**
 *******************************************************************************
 * @brief computeDirDistance-  get distance of B data set relative to A data set
 *
 *  <1> Parameter Description:
 *
 *  @param [In] Apoints, A data set
 *
 *  @param [In] Bpoints, B data set
 *
 *  @param [In] numA, size of A data set
 *
 *  @param [In] numB, size of B data set
 *
 *  @param [In] Range, ratio of part-Haustorff
 *
 *  @param [In] disType, type of Hausdorff distance
 *
 *  @return  distance.
 *
 *  <2> Detailed Description:
 *  if Range is between 0~1, it's part-Hausdorff; if Range is 1, it's ori-Hausdorff
 *  disType = 1, use maximum distance; disType = 0, use mean distance;
 *  \ingroup
 *******************************************************************************
 */
double computeDirDistance(const std::vector<Point3d_t>& Apoints,
                          const std::vector<Point3d_t>& Bpoints,
                          int numA, int numB, double Range, int distype);

/**
 *******************************************************************************
 * @brief meanDistance-  get mean distance
 *
 *  <1> Parameter Description:
 *
 *  @param [In] a[], array of distance values
 *
 *  @param [In] n, size of array
 *
 *  @return mean distance.
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
double meanDistance(double a[], int n);

/**
 *******************************************************************************
 * @brief sort-  sort distance values
 *
 *  <1> Parameter Description:
 *
 *  @param [In] a[], array of distance values
 *
 *  @param [In] n, size of array
 *
 *  @return mean distance.
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
void sort(double a[], int n);
/////////////////////////////////////////////////////////////////////////////

/**
 * Make sure specified path exists.
 *
 * i.e., if the specified path already exists nothing to do, otherwise create
 * all it and its all ancestor directories.
 *
 * @param strPath [IN] the path to make sure existence
 * @return true if it exists, false if failed to make sure it exist
 */
bool assurePathExists(const std::string &strPath);

/**
 *******************************************************************************
 * @brief getFilesFromFolder- get the files with special suffix names from the folder
 *
 *  <1> Parameter Description:
 *
 *  @param [In] folder - folder name
 *              suffix - suffix name
 *
 *  @param [out] files - the files with special suffix names from the input folder
 *
 *  @return mean distance.
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
bool getFilesFromFolder(const std::string &folder, const std::string &suffix, std::set<std::string> &files);

inline void convert2Float64Gps(uint32_t lon, uint32_t lat, uint32_t alt, Point3d_t& relGps)
{
    relGps.relLon = static_cast<float64_t>(lon) * 360 / pow(2, 32) - 180;
    relGps.relLat = static_cast<float64_t>(lat) * 180 / pow(2, 32) - 90;
    relGps.relAlt = static_cast<float64_t>(alt) / 1000 - 1000;

    return;
}

inline void convert2FixPointGps(Point3d_t& relGps, uint32_t &lon, uint32_t &lat, uint32_t &alt)
{
    lon = static_cast<uint32_t>((relGps.relLon + 180) * pow(2, 32) / 360 + 0.5);
    lat = static_cast<uint32_t>((relGps.relLat + 90) * pow(2, 32) / 180 + 0.5);
    alt = static_cast<uint32_t>((relGps.relAlt + 1000) * 1000 + 0.5);

    return;
}

inline void  displayVersion()
{
    COM_LOG_INFO << "Version: " << VERSION_INFO;
}

inline void  displayCmdLine(int argc,char *argv[])
{
    std::string cmdLine;
    for(int i=0; i< argc; ++i)
    {
        cmdLine.append(argv[i]);
        cmdLine.append("  ");
    }
    COM_LOG_CRITICAL << "module command: " << cmdLine;

    displayVersion();
}

inline void splitString(const std::string& src, const char * delim, std::vector<std::string>& dst)
{
     std::size_t pos = 0;
     int32_t len = static_cast<int32_t>(strlen(delim));
     std::size_t begin = pos;
     pos = src.find(delim,begin);

     while(std::string::npos != pos)
     {
         dst.push_back(src.substr(begin,pos-begin));
         begin = pos + len;
         pos = src.find(delim,begin);
     }

     if (!src.empty())
     {
          dst.push_back(src.substr(begin));
     }
}

inline void  combineString(const std::vector<std::string>& src,const char * delim, std::string& dst)
{
    if (src.empty())
        return;

    std::size_t index = 0;
    for(; index < src.size()-1; ++index)
    {
        dst.append(src[index]);
        dst.append(delim);
    }
    dst.append(src[index]);
}

std::string getDirName(std::string &dst, const std::string &src);

/**
 *******************************************************************************
 * @brief split-  splice elements in a vector into a string
 *
 *  <1> Parameter Description:
 *
 *  @param [In] passSegments, source segment id list
 *
 *  @param [In] strRecord, resulting string containing all the elements in passSegments separated by separate_character
 *
 *  @param [In] separate_character, specify the separator between each element
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
void splice(const std::vector<SegmentID_t> &passSegments, std::string& strRecord, const std::string& separate_character = ",");

void signalCatch();
void printBacktraceInfo(roadDBCore::int32_t signalValue) ;

void print(const std::vector<DBDivisionDetail_t>& divisions);

void print(const std::vector<std::shared_ptr<DBDivision_t>>& divisions);


}

#endif
