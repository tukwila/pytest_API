/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CacheDataExtracter.h
 * @brief  cache_data_extracter
 *******************************************************************************
 */
#pragma once

#include "BaseRoadDataInfo.h"
#include "RoadDataLayer.h"
#include "ThreadPool.h"
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

namespace RDBVehicleAPI
{
class BaseDataLoader;
class CacheDataExtracter;
typedef boost::function<void(SP(CacheDataExtracter))> cbExtracterCallBack_t;
class CacheDataExtracter : public boost::noncopyable, 
    public std::enable_shared_from_this<CacheDataExtracter>
{
public:
    // TO BE REMOVED
    // CacheDataExtracter(cbExtracterCallBack_t fCB, const segmentIDSet_t& added, 
    //     const segmentIDSet_t& decrease);

    CacheDataExtracter(BaseDataLoader* pDataLoader, const segmentIDSet_t& added, 
        const segmentIDSet_t& decrease);
    ~CacheDataExtracter();
public:
    bool startExtract();
	bool isNewDataExtracted(){return bNewDataExtracted_ == true;}
	const std::map<segmentID_t, SP(BaseRoadDataInfo)>& getExtractRoadData() {return pRoadInfos_;}
	
	const segmentIDSet_t& getDecreaseIds() {return removeSegmentIds_;}
private:
    bool extractOneSegment(segmentID_t segId);
private:
	std::mutex mtx_;
	cbExtracterCallBack_t extracterCB_;
	bool bNewDataExtracted_ = false;

    segmentIDSet_t addSegmentIds_;
    segmentIDSet_t removeSegmentIds_;
    seg2RoadDataPtrMap_t pRoadInfos_;

    BaseDataLoader* pDataLoader_;
};


// }
}