/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CompTrajSaver.h
 * @brief  the class for saving the offline comparision data.
 *******************************************************************************
 */
 
#pragma once
 
//*=============================================================================
//* section for including headers
 
//* include directive - local class header files
 
//* include directive - system header files
#include "algoInterface/Localization/LocCompTrajectory.h"
 
//* include directive - other lib header files
 
//* include directive - local project header files
 
//*=============================================================================
//* section for forward declaration
 
 
//*=============================================================================
//* section for namespace

//* beginning of namespace
namespace roadDBCore
{

class CompTrajSaver
{
public:
    CompTrajSaver();

    bool init(const std::string& prefixName);

    // private implementations for specific task
    void saveOfflineInfo(const algo::vehicle::LocCompTrajectory_t &compensatorOut);

    void finish();
private:
    void saveOfflineInfoHeader();

private:
    // member variables and etc
    std::string resultFileName_ = "_loc.out";
    std::string resultTmpFileName_ = "_loc.out.tmp";
};

} //* end of namespace