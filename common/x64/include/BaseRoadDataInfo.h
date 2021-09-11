/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   BaseRoadDataInfo.h
 * @brief  BaseRoadDataInfo 
 *******************************************************************************
 */

#pragma once

#include "InnerStructs.h"
#include "LaneRelation.h"
#include "DBConnect.h"
#include "CHorizon.h"
#include "Landscape.h"
#include "EVP.h"
#include "Paint.h"
#include "Node.h"
#include "Conf.h"
#include "RoadDataSummary.h"
#include "DBDirectory.h"
#include "RIVErrCode.h"
#include <boost/noncopyable.hpp>

namespace RDBVehicleAPI
{

#define GO_WITH_CHECKMODE(mode, returnRlt) \
 	if (mode == EDH_MODE_ACCEPTED) \
	{ \
    	continue; \
	} \
	else \
	{ \
    	return returnRlt; \
	} 

class BaseRoadDataInfo;
typedef std::map<segmentID_t, SP(BaseRoadDataInfo)> seg2RoadDataPtrMap_t;

typedef struct 
{
    float64_t minX;
    float64_t maxX;
    float64_t minY ;
    float64_t maxY;
} pointRangeResp_t;

template<typename L,typename N>
void connect(const L& line, const N& node, std::unordered_map<N, std::set<L>>& nodeToLines)
{
    nodeToLines[node].insert(line);
}


class BaseRoadDataInfo : public boost::noncopyable
{
protected:
	enum LANE_SIDE_E
	{
		LANE_SIDE_LEFT_E    = 0, 				
		LANE_SIDE_RIGHT_E	 = 1	
	} ;

public:    
    BaseRoadDataInfo(EXCEPTION_DATA_HANDLE_MODE mode = EDH_MODE_REFUSED);

    virtual ~BaseRoadDataInfo();
    //explicit BaseRoadDataInfo(segmentID_t id); 

public:
    virtual bool load(segmentID_t id, const SP_CONST(DBDirectory) pDBDirectory);
	bool init(const std::string& path);
	SP_CONST(RoadDataSummary) getSummaryRef() const;
	SP_CONST(DBDirectory) getDBDirectory() const;

    bool available();
    virtual void destroy();
    bool moveTo(const seg2RoadDataPtrMap_t& pRDs, const segmentIDSet_t& decIds);
    // virtual void doRuntimeCheck();

    void getRoadsInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const objectIDSet_t sourceRoadIDSet, OUT VSPVec(Landscape)& roadsVector) const;
    void getJunctionsInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const objectIDSet_t sourceJunctionIDSet, IN const std::map<objectID_t, int>& inRangRoadID2Exist, OUT VSPVec(Junction)& junctionsVector) const;
    bool getPassedJunctionIdsbySegId(const segmentID_t segId, objectIDSet_t& junctionIds) const;
    virtual bool getRoadIDsByDivisionID(const divisionID_t divisionId,objectIDSet_t &roadIDs) const;
	virtual void getDivisionIDsByRoadIDs(const objectIDSet_t roadIDs, divisionIDSet_t &divisionIDs) const;
	void getAllRoadInfo(VSPVec(Landscape)& roads) const;

	void getAllJunctionInfo(VSPVec(Junction)& junctions) const;
	const std::unordered_map<segmentID_t, objectIDSet_t>& getAllSegRoadIDs() const;

	virtual RIVErrCode_t getDataVersion(OUT std::string& majorVersion, OUT std::string& fixVersion) const {return RIV_COMN_UNSUPPORTED_CALL;}
	bool isSegmentInMasterDBNotInDir(const segmentID_t id) const;
	bool isRoadInMasterDBNotInDir(const objectID_t id) const;
	void setDiffRoadIDs(const objectIDSet_t& roadIds);
protected:
    //basic
    void setDiffSegIDs();
    bool loadRefencePoint(segmentID_t segId);
    bool loadLine(segmentID_t segId);
    bool loadCurve(segmentID_t segId);
    bool loadLanesConnect(segmentID_t segId);
	bool loadLanesNeighbour(segmentID_t segId);
	bool loadLanesChange(segmentID_t segId);
    bool loadLane(segmentID_t segId);
	bool loadLanesXSign(segmentID_t segId);
	bool loadLanesXRoadMark(segmentID_t segId);
    bool loadRoadTS(segmentID_t segId);
    bool loadRoadEdge(segmentID_t segId);
	bool loadBarrier(segmentID_t segId);
    bool loadRoad(segmentID_t segId);
    bool loadRoadNode(segmentID_t segId);
	bool loadIntersection(segmentID_t segId);
	bool loadIntersectionXRoad(segmentID_t segId);
	bool loadEVPAttribute(segmentID_t segId);
	bool loadEVP(segmentID_t segId);
	bool loadRoadMark(segmentID_t segId);
    //relation
	bool buildRelation();
	virtual bool buildRoadInfo() { return true; }
	void updateRoadInfo();
	virtual bool buildLane2Lane() { return true; }
	virtual bool buildLaneInfo() { return true; }
	bool buildJunctionInfo();  // fill road's ptrs
	virtual bool buildEvpConnect() { return true; }
	virtual bool buildLineInfo() { return true; }
	virtual bool buildPaintRelation() { return true; }
	virtual EDBType getDataType() const{return E_DB_TYPE_PB; }
	bool buildEvpInfo();
	
	void merge(const SP(BaseRoadDataInfo)& pRD);
	void merge(const std::map<segmentID_t, SP(BaseRoadDataInfo)>& pRDs);
	void erase(const segmentIDSet_t& decIds);

	//get info
    bool getLanesConnect(objectID_t laneID, SP(LaneConnection)& pLaneCnct)const;

	//bool getRoadMarkInfo(const objectID_t &id,SP(RoadMark)& pRoadMark);
	bool getNodeInfo(objectID_t nodeId, SP(Node)& pNode) const;
	// bool getAttributeInfo(const objectID_t &lineId, SP(EVPAttribute)& pEVPAttribute);

    bool isRoadInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const SP_CONST(Landscape)& pRoad) const;
    bool isLaneInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const SP_CONST(Lane)& pLane) const;
    bool isLineInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const SP_CONST(Line)& pLine) const;
    bool isCurveInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const SP_CONST(Curve)& pCurve) const;
    void getPointRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const WGS84_t& basicPoint, OUT pointRangeResp_t& pointRangeResp) const;
    bool isJunctionInRange(IN const WGS84_t& centerPoint, IN const float64_t range, IN const SP_CONST(Junction)& pJunction) const;
    void addNeighbourLanes(const objectID_t& neighbourLaneId, const SP(Lane)& pLane, const LANE_SIDE_E side, SP(Landscape)& pRoad);
public:
	//interfaces for ut
	bool getEVPInfo(const objectID_t& lineId, SP(EVP)& pLine) const;
	bool getJunctionInfo(objectID_t id, SP(Junction)& pJunction) const;
	bool getRoadInfo(objectID_t roadId, SP(Landscape)& pRoad) const;
	bool getLineInfo(const objectID_t& lineId, SP(Line)& pLine) const;
	bool getCurveInfo(objectID_t curveId, SP(Curve)& pCurve) const ;
	bool getLaneInfo(objectID_t laneId, SP(Lane)& pLane) const;
	bool getRoadEdgeInfo(objectID_t roadId, SP(InnerRoadEdges)& roadEdge) const;
	bool getTrafficSignInfo(const objectID_t& id, SP(TrafficSign)& pTrafficSign) const;

	std::shared_ptr<const Lane> doGetNearestLane(const NDSPoint_t& position, Pose_t* const pose,
                                                       float64_t* distance) const;
protected:
	bool                            bAvailable_ = false;
	//db handler
	SP(DBConnect)       				pDBConnect_;

	//BasicInfo
	std::unordered_map<segmentID_t, WGS84_t>           seg2BasicPoint_;

	//paint
	std::unordered_map<objectID_t, SP(Paint)>      allPaintsInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>      seg2PaintIdSet_;

	//curves
	std::unordered_map<objectID_t, SP(Curve)>      allCurvesInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>      seg2CurveIdSet_;

	//Line
	std::unordered_map<objectID_t, SP(Line)>                   allLinesInfo_;
	std::unordered_map<objectID_t, objectID_t>                   allLinesBasicInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>               seg2LineIdSet_;
	std::unordered_map<objectID_t, SP(InnerEVP)> 				   	    allEVPsBasicInfo_;
	std::unordered_map<objectID_t, SP(EVP)>			   	   		allEVPsInfo_;
	std::multimap<objectID_t, SP(EVPAttribute)>  			allEVPAttributesInfo_;

	//Lane   
	std::unordered_map<objectID_t, SP(InnerLane)>                         allLanesBasicInfo_;
	std::unordered_map<objectID_t, SP(Lane)> 						allLanesInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>                seg2LaneIdSet_;


	//TS
	std::unordered_map<objectID_t, SP(TrafficSign)>                     allRoadTSInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>               seg2RoadTSIdSet_;

	//RoadMarks
	//std::unordered_map<objectID_t, SP(RoadMark)>                   allRoadMarkInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>            seg2RoadMarkIdSet_;

	//junction
	std::unordered_map<objectID_t, SP(Node)>                  	allNodesInfo_;

	//Road
	std::unordered_map<segmentID_t, objectIDSet_t>                 seg2RoadIdSet_;
	std::unordered_map<objectID_t, SP(InnerRoadEdges)>                  allRoadEdgeInfo_;
	std::unordered_map<objectID_t, SP(InnerRoad)>                     allRoadsBasicInfo_;
	std::unordered_map<objectID_t, SP(Landscape)>       			    allRoadsInfo_;
	std::unordered_map<objectID_t, divisionIDSet_t>  			road2DivisionIdSet_;
	std::unordered_map<divisionID_t,objectIDSet_t> 			  division2RoadIdSet_;
	//laneconnect
	LaneRelation                 	   laneRelation_;
	std::unordered_map<objectID_t, SP(LaneConnection)>              allLaneConnectInfo_;

	//Intersection, we name junction in common api
	std::unordered_map<objectID_t, SP(InnerJunction)>             allJunctionsBasicInfo_;
	std::unordered_map<objectID_t, SP(Junction)>			      allJunctionsInfo_;
	std::unordered_map<segmentID_t, objectIDSet_t>			   seg2JunctionIdSet_;
	EXCEPTION_DATA_HANDLE_MODE      dataCheckMode_;
	SP(RoadDataSummary) pSummary_;
	SP(DBDirectory)     pDBDirectory_;
	segmentIDSet_t segIDsInMasterNotInDir_; // segIDS in master db but not in data db
	objectIDSet_t roadIDsInMasterNotInDir_; // segIDS in master db but not in data db
	
	map<objectID_t, vector<objectID_t>> nurbsExpressId2LineIdListMap_;
	map<objectID_t, objectID_t> lineId2PaintIdMap_;


};

}
