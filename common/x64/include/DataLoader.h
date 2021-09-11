/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LoadMode.cpp
 * @brief  load mode handler
 *******************************************************************************
 */

#pragma once

#include "VehicleAPICommon.h"
#include "RIVErrCode.h"
#include "RoadDataLayer.h"
#include "BaseRoadDataInfo.h"
#include "utilsSys.h"
#include "RWLock.h"
#include <boost/thread/thread.hpp>
#include <atomic>
namespace RDBVehicleAPI
{
    
class BaseDataLoader {
public:
    BaseDataLoader(const conf_t& conf) ;
    virtual ~BaseDataLoader(){}
    virtual RIVErrCode_t parseParam(); 

    virtual bool startLoading() = 0;
    //virtual RIVErrCode_t doReport(const segmentID_t segId) { return RIV_COMN_SUCCESS; } 
    virtual RIVErrCode_t doReport(const WGS84_t& gps, updatePositionCallBack_t* cb) { return RIV_COMN_UNSUPPORTED_CALL; } 
    virtual void stop(); 
    virtual bool updateDataLoaded(const seg2RoadDataPtrMap_t& pRDs, const segmentIDSet_t& decIds);
    virtual void setDiffRoadIDs() = 0;
    bool isDataLoaded() const;
    void lock(const RWLOCKOP_E op);
    SP_CONST(BaseRoadDataInfo) getRoadDataInfo() const;
    SP_CONST(DBDirectory) getDBDirectory() const;
    const conf_t& getConf() const;
    
protected:
    volatile bool bDataLoaded_;
    conf_t conf_;
    SP(BaseRoadDataInfo) pRoadInfo_;
    	
private:
    RWLock rwLocker_;
};

class WholeDataLoader : public BaseDataLoader {
public:
    WholeDataLoader(const conf_t& conf) : BaseDataLoader(conf) {}
    ~WholeDataLoader(){};
    bool startLoading() override;
    void setDiffRoadIDs() override;
};

class RealTimeDataLoader : public BaseDataLoader {
public:
    RealTimeDataLoader(const conf_t& conf) : BaseDataLoader(conf)
    ,initialSegmentId_(0)
    ,bRunning_(false)
    {
        pLayer_ = std::make_shared<RoadDataLayer>(conf_.refreshConf);
        pCacheLayer_ = std::make_shared<RoadDataLayer>(conf_.refreshConf);
    }

    ~RealTimeDataLoader();
    RIVErrCode_t parseParam() override;
    bool startLoading() override;
    RIVErrCode_t doReport(const WGS84_t& gps, updatePositionCallBack_t* cb) override;
    void stop() override;
    bool updateDataLoaded(const seg2RoadDataPtrMap_t& pRDs, const segmentIDSet_t& decIds) override;
    void setDiffRoadIDs() override;
private:
    void cacheThreadFuncLoop();
    void extractBySegment(const segmentID_t& segId);
    // RIVErrCode_t doReport(const segmentID_t segId);
    // bool shouldReport(const segmentID_t segId);

private:
    segmentID_t initialSegmentId_;
    std::atomic<bool> bRunning_;
	std::shared_ptr<boost::thread> pThread_;
    boost::thread::attributes attr_;
    SP(RoadDataLayer) pLayer_;
    SP(RoadDataLayer) pCacheLayer_;
    // segmentID_t lastSegmentId_;
    // uint64_t lastRecvGpsTime_;              // The time of receiving gps last time.
    // uint64_t gpsCheckInterval_;             // The interval of checking gps reported.
};
}