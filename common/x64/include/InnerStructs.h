/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   innerTYpe.h
 * @brief  some define for inner use
 *******************************************************************************
 */
#pragma once

#include <unordered_map>
#include <vector>
#include "VehicleAPICommon.h"
#include "RoadEdge.h"
#include "Geo.h"
#include "VehicleAPIError.h"

namespace RDBVehicleAPI
{
class InnerBaseWithConnectedIds
{
public:
   void addFromObjectId(const objectID_t &id) { fromObjectIds_.insert(id); }
   void addToObjectId(const objectID_t &id) { toObjectIds_.insert(id); }
   std::set<objectID_t> &getFromObjectIds() { return fromObjectIds_; }
   std::set<objectID_t> &getToObjectIds() { return toObjectIds_; }

private:
   std::set<objectID_t> fromObjectIds_;
   std::set<objectID_t> toObjectIds_;
};

class InnerJunction
{
public:
   InnerJunction(const objectID_t &id, const objectIDSet_t &edgeIds);
   InnerJunction(const objectID_t &id);
   objectIDSet_t getEdgeIds() const;
   objectIDSet_t getRoadIds() const;
   void addRoadId(const objectID_t &id);

private:
   objectID_t junctionId_;
   objectIDSet_t edgeIds_;
   objectIDSet_t roadIds_{};
};

class InnerRoad : public InnerBaseWithConnectedIds
{
public:
   InnerRoad(const objectID_t &id, const objectID_t &fromNodeId,
             const objectID_t &toNodeId, const float64_t length);
   objectID_t getToNodeId() const;
   objectID_t getFromNodeId() const;

private:
   objectID_t roadId_;
   objectID_t fromNodeId_; // we call junction node in common api
   objectID_t toNodeId_;
   float64_t length_ = 0;
   // std::vector<objectID_t> fromRoadIds_;
   // std::vector<objectID_t> toRoadIds_;
};
enum MARKERTYPE_E : uint8_t
{
   MARKERTYPE_SOLID_E = 0,
   MARKERTYPE_DASHED_E = 1,
   MARKERTYPE_IMPUTED_E = 2,
   MARKERTYPE_SLAMTRACE_E = 3,
   MARKERTYPE_UNLABELED_E = 4,
   MARKERTYPE_DOUBLE_SOLID_E = 5,
   MARKERTYPE_DOUBLE_DASHED_E = 6,
   MARKERTYPE_DASHED_SOLID_E = 7,
   MARKERTYPE_SOLID_DASHED_E = 8,
   MARKERTYPE_DASHED_DASHED_DASHED_E = 9,
   MARKERTYPE_DASHED_SOLID_DASHED_E = 10,
   MARKERTYPE_EVP_E = 11,
   MARKERTYPE_ROAD_EDGE_E = 12,
   MARKERTYPE_MAX_E = 13
};

// NURBS EquationDescription
class NurbsDesc
{
 public:
    void setPaintTotalLength(const float64_t  paintTotalLength);
    void setLineLength(const float64_t  LineLength);
    void addPaintEndPoint(const std::vector<float64_t>& endPoint);
    void addKnot(const float64_t point);
    void addControlPoint(const point3D_t& point);
    const std::vector<point3D_t>&  getControlPoints() const;
    const std::vector<float64_t>&  getKnots() const;
    const std::vector< std::vector<float64_t> >& getPaintEndPoints() const;
    float64_t getPaintTotalLength() const;
    float64_t getLineLength() const;
 private:
    std::vector<point3D_t>  controlPoints_;
    std::vector<float64_t>  knots_;
    std::vector< std::vector<float64_t> >  paintEndPoints_;
    float64_t paintTotalLength_ = 0; /*if not find,value is 0*/
    float64_t lineLength_ = 0;  /*if not find,value is 0 */
};

typedef int32_t lineIndex_t;
/*class InnerCurve
{
public:
	InnerCurve(const objectID_t& id, const objectID_t& lineId, const lineIndex_t lineIndex, 
		const MARKERTYPE_E markerType, const float64_t length, const NurbsDesc&  params);
	void setId(const objectID_t& id);
	void setLineId(const objectID_t& lineId);
	void setLineIndex(const lineIndex_t idx);
	void setMarkerType(const MARKERTYPE_E type);
	void setLength(const float64_t length);
	void setNurbs(const NurbsDesc& nurbs);
private:
	objectID_t			id_;
	objectID_t         lineId_;
	lineIndex_t      lineIndex_ = DEFAULT_LINEINDEX;    
	MARKERTYPE_E     markerType_ = MARKERTYPE_SOLID_E;   		
	float64_t       length_ = 0;
	NurbsDesc       params_;         					
} ;*/

class InnerLane : public InnerBaseWithConnectedIds
{
public:
   InnerLane(const objectID_t &id, const objectID_t &roadId,
             const objectID_t &lLineID, const objectID_t &rLineID,
             const objectID_t &avgSlamTraceID);

public:
   objectID_t getId() const;
   objectID_t getRoadId() const;
   objectID_t getLeftLineId() const;
   objectID_t getRightLineId() const;
   objectID_t getAvgSlamTraceId() const;
   objectIDSet_t getTrafficSignIds() const;
   objectIDSet_t getRoadMarkIds() const;
   void addTrafficSignId(const objectID_t &signId);
   void addRoadMarkId(const objectID_t &markId);

   void setLeftLaneId(const objectID_t &laneId) { leftLaneId_ = laneId; }
   void setRightLaneId(const objectID_t &laneId) { rightLaneId_ = laneId; }

   objectID_t getLeftLaneId() const { return leftLaneId_; }
   objectID_t getRightLaneId() const{ return rightLaneId_; }

private:
   objectID_t id_;
   objectID_t roadId_;
   objectID_t lLineID_;
   objectID_t rLineID_;
   objectID_t avgSlamTraceID_;
   objectIDSet_t trafficSigns_;
   objectIDSet_t roadMarkIds_;
   objectID_t leftLaneId_ = INVALID_OBJECT_ID;
   objectID_t rightLaneId_ = INVALID_OBJECT_ID;
};

enum LANE_CONNECT_DIR_E
{
   LANE_CONNECT_DIR_FROM_E = 0,
   LANE_CONNECT_DIR_TO_E = 1,
   LANE_CONNECT_DIR_LEFT_E = 2,
   LANE_CONNECT_DIR_RIGHT_E = 3
};

class InnerRoadEdges
{
public:
   void addLeftEdge(const SP(RoadEdge) & edge);
   void addRightEdge(const SP(RoadEdge) & edge);
   VSPVec(RoadEdge) getLeftEdges() const;
   VSPVec(RoadEdge) getRightEdges() const;

private:
   VSPVec(RoadEdge) leftEdges_;
   VSPVec(RoadEdge) rightEdges_;
};

class LaneConnection
{
public:
   void setId(const objectID_t &id);
   void setFromLaneIds(const objectIDSeq_t &laneIds);
   void setToLaneIds(const objectIDSeq_t &laneIds);
   void setLeftLaneId(const objectIDSeq_t &laneId);
   void setRightLaneId(const objectIDSeq_t &laneId);
   void setLeftChangeLaneId(const objectIDSeq_t &laneId);
   void setRightChangeLaneId(const objectIDSeq_t &laneId);

   objectIDSet_t getFromLaneIds() const;
   objectIDSet_t getToLaneIds() const;
   objectID_t getLeftLaneId() const; // empty
   objectID_t getRightLaneId() const;
   objectID_t getLeftChangeLaneId() const;
   objectID_t getRightChangeLaneId() const;

private:
   objectID_t id_;
   objectIDSet_t fromLaneIds_;
   objectIDSet_t toLaneIds_;
   objectID_t leftLaneId_ = "";
   objectID_t rightLaneId_ = "";
   objectID_t leftChangeLaneId_ = "";
   objectID_t rightChangeLaneId_ = "";
};

class InnerEVP : public InnerBaseWithConnectedIds
{
public:
   InnerEVP(const objectID_t &lineId, const objectID_t &fromNodeId,
            const objectID_t &toNodeId, const objectID_t &laneId);
   InnerEVP(const objectID_t &lineId);

public:
   objectID_t getLaneId() const;
   objectID_t getFromNodeId() const;
   objectID_t getToNodeId() const;

private:
   objectID_t lineId_ = INVALID_OBJECT_ID;
   objectID_t fromNodeId_ = INVALID_OBJECT_ID;
   objectID_t toNodeId_ = INVALID_OBJECT_ID;
   objectID_t laneId_ = INVALID_OBJECT_ID;
};

// }
} // namespace RDBVehicleAPI
