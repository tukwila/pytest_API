/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   PerformanceAnalysis.h
 * @brief  class for PerformanceAnalysis
 *******************************************************************************
 */

#ifndef PERFORMANCE_ANALYSIS
#define PERFORMANCE_ANALYSIS

#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <boost/date_time.hpp>
#include <boost/random.hpp>
#include "typeDef.h"


namespace roadDBCore
{ 

inline uint64_t getTickCount()
{
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    return boost::numeric_cast<uint64_t>(now.time_of_day().total_milliseconds());
}
    
struct ScopeTictac
{
    ScopeTictac(uint64_t& startTick0, uint64_t& endTick0, std::string& strFunction);    
    ~ScopeTictac();

    uint64_t& startTick;
    uint64_t& endTick;
    std::string& strFuncName;
};

class PerformanceAnalysis
{
private:
    struct TickInfo
    {
        TickInfo(const std::string& strFunction): startTick(0LL), endTick(0LL), strFuncName(strFunction)
        {
        }
        
        uint64_t startTick;
        uint64_t endTick;
        std::string strFuncName;
    };
    
    struct SumInfo
    {
        SumInfo(): sumTick(0LL), numCall(0)
        {}

        SumInfo(uint64_t sumTick, uint32_t numCall): sumTick(sumTick), numCall(numCall)
        {}

        uint64_t sumTick;
        uint32_t numCall;
    };

public:
    PerformanceAnalysis(const std::string &strModuleName):strModuleName_(strModuleName) {}
    ~PerformanceAnalysis();
    ScopeTictac getScopeTictac(const std::string& strFuncName);    

    void reset() {vecTicks_.clear();}
    void dump(uint64_t totalTimeMS = 0);
    void dumpTotal(uint64_t totalTimeMS = 0);


private:
    void calTimeInVecTicks(uint64_t &totalTicks, std::vector<uint64_t> &vecUsedTicks);
    void clearVecTicks();

    std::vector<TickInfo*> vecTicks_;
    std::map<std::string, SumInfo> mapHistoryTicks_;
    std::string strModuleName_;
};


}// namespace roadDBCore




#endif //PERFORMANCE_ANALYSIS
