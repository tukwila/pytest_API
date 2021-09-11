/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   PerformAnalysisManager.h
 * @brief  class for PerformAnalysisManager
 *******************************************************************************
 */

#ifndef  PERFORM_ANALYSIS_MANAGER_H_
#define  PERFORM_ANALYSIS_MANAGER_H_

#include <map>
#include <string>
#include "PerformanceAnalysis.h"

namespace roadDBCore
{


class PerformAnalysisManager
{

private:
    PerformAnalysisManager();

public:
    static PerformAnalysisManager *getInstance( );
    static void releaseInstance();
    PerformanceAnalysis &getPfmAlsInstance(const std::string &strModuleName);

private:
    static PerformAnalysisManager *s_pManager_;
    static std::map<std::string, PerformanceAnalysis *> s_mapPerformAnalysises_;
};



}// namespace roadDBCore




#endif //PERFORM_ANALYSIS_MANAGER_H_

