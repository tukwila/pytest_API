/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   PerformAnalysisInterface.h
 * @brief  interface for PerformanceAnalysis module
 *******************************************************************************
 */

#ifndef  PERFORM_ANALYSIS_INTERFACE_H_
#define  PERFORM_ANALYSIS_INTERFACE_H_

#include "PerformanceAnalysis/PerformAnalysisManager.h"

#ifdef ENABLE_LOG

#define MONITOR_PERFORMANCE(strModuleName, strFuncName) roadDBCore::ScopeTictac localTempTictac = \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).getScopeTictac(strFuncName);
#define MONITOR_FUNCTION_PERFORMANCE(strModuleName) roadDBCore::ScopeTictac localTempTictac = \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).getScopeTictac(__FUNCTION__);
#define DUMP_PERFORMANCE(strModuleName) \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).dump();
#define DUMP_PERFORMANCE_TOTAL(strModuleName) \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).dumpTotal();
#define RESET_PERFORMANCE(strModuleName) \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).reset();

#define DUMP_PERFORMANCE_EX(strModuleName, totalTimeMS) \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).dump(totalTimeMS);
#define DUMP_PERFORMANCE_TOTAL_EX(strModuleName, totalTimeMS) \
        roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).dumpTotal(totalTimeMS);

#else

#define MONITOR_PERFORMANCE(strModuleName, strFuncName)
#define MONITOR_FUNCTION_PERFORMANCE(strModuleName)
#define DUMP_PERFORMANCE(strModuleName)
#define DUMP_PERFORMANCE_TOTAL(strModuleName)
#define RESET_PERFORMANCE(strModuleName)

#endif


// Macro for release memory
#define RELEASE_PERFORMANCE_MEMORY()                  roadDBCore::PerformAnalysisManager::releaseInstance()

// Macros for server
#define MONITOR_PERFORMANCE_SERVER_ALGO(strFuncName)  MONITOR_PERFORMANCE("SERVER_ALGO", strFuncName)
#define MONITOR_FUNCTION_PERFORMANCE_SERVER_ALGO()    MONITOR_FUNCTION_PERFORMANCE("SERVER_ALGO")
#define DUMP_PERFORMANCE_SERVER_ALGO()                DUMP_PERFORMANCE("SERVER_ALGO")
#define DUMP_PERFORMANCE_TOTAL_SERVER_ALGO()          DUMP_PERFORMANCE_TOTAL("SERVER_ALGO")
#define RESET_PERFORMANCE_SERVER_ALGO()               RESET_PERFORMANCE("SERVER_ALGO")

// Macros for vehicle TS
#define MONITOR_PERFORMANCE_VEHICLE_TS(strFuncName)  MONITOR_PERFORMANCE("VEHICLE_TS", strFuncName)
#define MONITOR_FUNCTION_PERFORMANCE_VEHICLE_TS()    MONITOR_FUNCTION_PERFORMANCE("VEHICLE_TS")
#define DUMP_PERFORMANCE_VEHICLE_TS()                DUMP_PERFORMANCE("VEHICLE_TS")
#define DUMP_PERFORMANCE_TOTAL_VEHICLE_TS()          DUMP_PERFORMANCE_TOTAL("VEHICLE_TS")
#define RESET_PERFORMANCE_VEHICLE_TS()               RESET_PERFORMANCE("VEHICLE_TS")

// Macros for vehicle ROAD
#define MONITOR_PERFORMANCE_VEHICLE_ROAD(strFuncName)  MONITOR_PERFORMANCE("VEHICLE_ROAD", strFuncName)
#define MONITOR_FUNCTION_PERFORMANCE_VEHICLE_ROAD()    MONITOR_FUNCTION_PERFORMANCE("VEHICLE_ROAD")
#define DUMP_PERFORMANCE_VEHICLE_ROAD()                DUMP_PERFORMANCE("VEHICLE_ROAD")
#define DUMP_PERFORMANCE_TOTAL_VEHICLE_ROAD()          DUMP_PERFORMANCE_TOTAL("VEHICLE_ROAD")
#define RESET_PERFORMANCE_VEHICLE_ROAD()               RESET_PERFORMANCE("VEHICLE_ROAD")

// Macros for vehicle PAINTING DETECTION DL
#define MONITOR_PERFORMANCE_VEHICLE_PAINTING_DETECTION_DL(strFuncName)  MONITOR_PERFORMANCE("VEHICLE_PAINTING_DETECTION_DL", strFuncName)
#define MONITOR_FUNCTION_PERFORMANCE_VEHICLE_PAINTING_DETECTION_DL()    MONITOR_FUNCTION_PERFORMANCE("VEHICLE_PAINTING_DETECTION_DL")
#define DUMP_PERFORMANCE_VEHICLE_PAINTING_DETECTION_DL()                DUMP_PERFORMANCE("VEHICLE_PAINTING_DETECTION_DL")
#define DUMP_PERFORMANCE_TOTAL_VEHICLE_PAINTING_DETECTION_DL()          DUMP_PERFORMANCE_TOTAL("VEHICLE_PAINTING_DETECTION_DL")
#define RESET_PERFORMANCE_VEHICLE_PAINTING_DETECTION_DL()               RESET_PERFORMANCE("VEHICLE_PAINTING_DETECTION_DL")

#if 0

inline roadDBCore::ScopeTictac CreateTictac(const std::string &strModuleName, const std::string &strFuncName)
{
    return roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).getScopeTictac(strFuncName);
}

inline roadDBCore::ScopeTictac CreateFunctionTictac(const std::string &strModuleName)
{
    return CreateTictac(strModuleName, __FUNCTION__);
}

inline void DumpPerformAnalysis(const std::string &strModuleName, uint32_t number)
{
    roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).dump(number);
}

inline void ResetPerformAnalysis(const std::string &strModuleName)
{
    roadDBCore::PerformAnalysisManager::getInstance()->getPfmAlsInstance(strModuleName).reset();
}

#define MORNITOR_PERFORMANCE(strModuleName, strFuncName) roadDBCore::ScopeTictac localTempTictac = CreateTictac(strModuleName, strFuncName);
#define MORNITOR_FUNCTION_PERFORMANCE(strModuleName) roadDBCore::ScopeTictac localTempTictac = CreateTictac(strModuleName, __FUNCTION__);
#define DUMP_PERFORMANCE(strModuleName, number) DumpPerformAnalysis(strModuleName, number);
#define DUMP_PERFORMANCE1(strModuleName) DumpPerformAnalysis(strModuleName, 1);
#define RESET_PERFORMANCE(strModuleName) ResetPerformAnalysis(strModuleName);

#endif


#endif //PERFORM_ANALYSIS_INTERFACE_H_


