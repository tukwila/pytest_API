/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RoadDataLayer.h
 * @brief  road_data_layer 
 *******************************************************************************
 */
#pragma once

#include <mutex>
#include <condition_variable>
#include "InnerStructs.h"
#include <boost/noncopyable.hpp>
#include "VehicleAPICommon.h"
#include "RIVErrCode.h"
#include "TSQueue.hpp"
#include <functional>
namespace RDBVehicleAPI
{
#define INIT_LAYER_ID 0
#define IVALID_LAYER_ID -1
typedef std::function<void()> updatePositionCallBack_t;
class RoadDataLayer;

typedef std::map<segmentID_t, layerIndex_t> segment2LayIndexMap_t;
typedef std::map<layerIndex_t, segmentIDUnorderSet_t> layerIndex2SegmentMap_t;

class RoadDataLayer : public boost::noncopyable
{
public:    
    RoadDataLayer(const refreshConf_t& refreshConf);
    RoadDataLayer();    
    virtual ~RoadDataLayer();
public:
    void reset();
    segmentID_t pop();
    bool sendStopNotify();
    void doMove(const SP(RoadDataLayer)& pMoveTo);
    void diff(const RoadDataLayer& other,segmentIDSet_t& added,segmentIDSet_t& decrease);
	const segmentID_t& getCenterID() const  {return centerID_;}
	const segmentIDSet_t& getAllSegIDs() const {return allSegIDs_;}
	const segment2LayIndexMap_t& getSegment2Layer() const {return segment2Layer_;}
    void rebuildBySegmentId(const segmentID_t segId, const segmentIDSet_t& dbExistIds);
    // void setLastSegmentId(const segmentID_t segId);

    void snapShot();
    
    RIVErrCode_t doReport(const WGS84_t& gps, const segmentIDSet_t& dbExistIds, updatePositionCallBack_t* cb);
    updatePositionCallBack_t* getUpdatePositionCallback();
private:
    void build(const segmentIDSet_t& dbExistIds);
    segmentID_t getToUpdateID();
    bool needUpdate(const segmentID_t segmentId);
    RIVErrCode_t shouldReport(const segmentID_t segId);
    //RIVErrCode_t doReport(const segmentID_t reportId);
    RIVErrCode_t push(const segmentID_t reportId, updatePositionCallBack_t* cb);
    
private:
    std::mutex mtxLayer_;
    std::condition_variable condLayer_;

    TSQueue<segmentID_t> reportIds_;
    bool bStopUpdate_ = false;
	
    segmentID_t centerID_ = INIT_LAYER_ID; 
    segmentIDSet_t allSegIDs_;
    segment2LayIndexMap_t segment2Layer_;

    segmentID_t lastSegmentId_ = 0;
    uint64_t lastRecvGpsTime_ = 0;              // The time of receiving gps last time.

    updatePositionCallBack_t *callback_ = nullptr;
    refreshConf_t refreshConf_;
};

// }
}
