/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SRManager.h
 * @brief  SRManager 
 *******************************************************************************
 */
#pragma once
#include <boost/noncopyable.hpp>
#include "VehicleAPICommon.h"
#include "SRDef.h"
#include "utility/Singleton.h"

namespace RDBVehicleAPI
{
typedef uint32_t        tsType_t;
typedef int32_t        srValue_t;

class SRManager
{
public:
    SRManager();
    ~SRManager();

public:
    bool get(const SR_TYPE_E& type, srValue_t& value, std::string& desc);
    
private:

    // void init();

    void initSL();

    void initOther();
    
    void initSLItem(SR_TYPE_E type, srValue_t slValue);

    void initOtherItem(SR_TYPE_E type, const std::string& desc);
	
    
private:
    std::map<SR_TYPE_E, srValue_t>  speedLimits_;
    std::map<SR_TYPE_E, std::string >  otherSigns_;
};
#define SRMgr  roadDBCore::Singleton<SRManager>::getInstance()
}
