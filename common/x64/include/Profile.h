/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Path.h
 * @brief  The class of horizon path, in accordance with ADASIS v3 Spec.
 *
 * Change Log:
 * Date              Author            Changes
 * 2020-03-01        Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include "LogicTypes.h"
#include "Lane.h"
#include <memory>

namespace RDBVehicleAPI
{
/**
 * @brief  The class definition for objects.
 */
const double CURVATURE_THRESH = 0.002;
const double DIRECTION_CHANGE_THRESH = 0.02;

enum ProfileType
{
    ProfileType_None = 0,
    ProfileType_Node = 1,
    ProfileType_LaneModel = 4, 
    ProfileType_LaneConnectivity = 5,
    ProfileType_LinearObjects = 6,
    ProfileType_LanesGeometry = 7,
    ProfileType_LaneWidth = 8,
    ProfileType_Curvature = 22,
    ProfileType_TrafficSign = 28,
    ProfileType_EffectiveSpeedLimit = 31
};

class Profile
{
public:
    uint32_t getID() const { return profileID_; }
    uint16_t getPathID() const { return pathID_; }
    uint32_t getOffset() const { return offset_; }
    uint32_t getEndOffset() const { return endOffset_; }
    virtual ProfileType getType() const = 0;

    void setOffset(uint32_t offset) { offset_ = offset; }
    void setEndOffset(uint32_t endOffset) { endOffset_ = endOffset; }
    Profile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset) : profileID_(id), pathID_(pathID), offset_(offset), endOffset_(endOffset) {}
    virtual ~Profile() = default;

private:
    uint32_t profileID_;
    uint16_t pathID_;
    uint32_t offset_;    // the unit is centi meter
    uint32_t endOffset_; // the unit is centi meter
    ProfileType type_;
};

enum RightOfWay
{
    Unknown = 0,
    MustYeild = 1,
    HasRightOfWay = 2
};

struct NodeArm
{
    uint16_t subPathID;
    float32_t probability;
    float32_t turnAngle;
    bool isComplexIntersection;
    RightOfWay right;
};

class NodeProfile : public Profile
{
public:
    const std::vector<NodeArm>& getAllArms() const { return arms_; }
    NodeArm getMPArm(); // get the most probable node arm
    virtual ProfileType getType() const {return ProfileType_Node;}

    NodeProfile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset) : Profile(id, pathID, offset, endOffset) {}
    void addNodeArm(uint16_t pathID, float32_t probability, float32_t angle, RightOfWay right, bool isComp = false) { arms_.push_back({pathID, probability, angle, isComp, right}); }

private:
    std::vector<NodeArm> arms_;
    friend class Path;
    friend class Horizon;
};

enum TrafficSignType
{
    TrafficSign_Stop = 33,
    TrafficSign_SpeedLimit = 87
};

class TrafficSignProfile : public Profile
{
public:
    TrafficSignType getSignType() const { return signType_; }
    uint32_t getValue() const { return value_; }
    int32_t getShift() const { return shift_; }
    virtual ProfileType getType() const {return ProfileType_TrafficSign;}
    TrafficSignProfile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset,
                       TrafficSignType signType, uint32_t value, int32_t shift) : Profile(id, pathID, offset, endOffset), signType_(signType) {}

private:
    TrafficSignType signType_;
    uint32_t value_ = 0;
    int32_t shift_ = 0; // unit is centi meter
    friend class Path;
};

class CurvatureProfile : public Profile
{
public:
    const std::vector<std::pair<uint32_t, float32_t>>& getCurvatureArray() const { return curvatureArray_; }
    virtual ProfileType getType() const {return ProfileType_Curvature;}

    CurvatureProfile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset) : Profile(id, pathID, offset, endOffset) {}
    void addCurvaturePair(uint32_t offset, float32_t curvature) { curvatureArray_.push_back(make_pair(offset, curvature)); }

private:
    std::vector<std::pair<uint32_t, float32_t>> curvatureArray_;
    friend class Path;
};

/*
A lane has defined start and end through opening or closing part, 
alternatively through splitting of a lane in two or more lanes respectively 
merging two or more lanes in one lane.
*/
enum LaneTransition
{
    LaneTransition_None = 0,
    LaneTransition_Opening = 1,//width changes continuously and has one nextLane
    LaneTransition_Closing = 2,
    LaneTransition_Merging = 3,
    LaneTransition_Splitting = 4//width changes continuously and has more than one nextLane
};

 /*
 The Lane Info structure represents the basic description of a lane, 
 and it is a part of each Lane Model profile entry.
 Lanes are counted beginning with 1 from outmost to midmost in the direction of the lane
 */
struct LaneInfo
{
    uint8_t index;   
    LaneTransition transition;  // not implemented now
    objectID_t centerLineID;
    objectID_t leftBoundaryID;
    objectID_t rightBoundaryID;
    LaneInfo(uint8_t _index, LaneTransition _transition, objectID_t cId, objectID_t lbID,
    objectID_t rbID):index(_index), transition(_transition), centerLineID(cId), leftBoundaryID(lbID), rightBoundaryID(rbID){}
};
/*
A Lane Model entry shall represent a stretch of road with the same set of lanes, 
no physical changes affecting a vehicle’s transition between these lanes, and no 
legal changes affecting the transitions between the lanes.
*/
class LaneModelProfile : public Profile
{
public:
    // uint8_t getNumberOfLanes(){return laneInfos_.size();}
    std::shared_ptr<const LaneInfo> getLaneInfo(const uint8_t index) ;
    const std::vector<std::shared_ptr<const LaneInfo>>& getAllLaneInfos() const { return laneInfos_;};

    ProfileType getType() const override {return ProfileType_LaneModel;}
    LaneModelProfile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset) : Profile(id, pathID, offset, endOffset) {}
    void addLaneInfo(const std::shared_ptr<const LaneInfo>&  lane) { laneInfos_.push_back(lane); }

private:
    std::vector<std::shared_ptr<const LaneInfo>> laneInfos_;
    friend class Path;
};

struct LaneConnectivityPair 
{ 
    uint8_t initialLaneNumber;
    uint16_t initialPathID;
    uint8_t newLaneNumber;
    uint16_t newPathID;
    LaneConnectivityPair(uint8_t _initialLaneNumber, uint16_t _initialPathID, uint8_t _newLaneNumber,  uint16_t _newPathID)
        :initialLaneNumber(_initialLaneNumber), initialPathID(_initialPathID),newLaneNumber(_newLaneNumber), newPathID(_newPathID){}
};

/*
The Lane Connectivity profile contains a list of connectivity pairs that show how the lanes described 
in the Lane Model profile are reachable from the lanes defined previously on the same path or on different paths.
This list does not include lanes which can be reached through a regular lane change

one LaneConnectivityPair for each combination 
*/
class LaneConnectivityProfile : public Profile
{
public:
    const std::vector<std::shared_ptr<const LaneConnectivityPair>>& getLaneConnectivities() const {return connectivityPairs_;}

    ProfileType getType() const override {return ProfileType_LaneConnectivity;}

    LaneConnectivityProfile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset) : Profile(id, pathID, offset, endOffset) {}
    void addConnectivity(const std::shared_ptr<const LaneConnectivityPair>&  lane) { connectivityPairs_.push_back(lane); }

private:
    std::vector<std::shared_ptr<const LaneConnectivityPair>> connectivityPairs_;
    friend class Path;
    friend class Horizon;
};

enum LinearObjectType {
    Linear_Object_Centerline,
    Linear_Object_LaneMarking,
    Linear_Object_Guardrail,
    Linear_Object_Fence,
    Linear_Object_Kerb,
    Linear_Object_Wall
};

enum LineMarkingType {
    Line_Marking_Unknown,
    Line_Marking_None,
    Line_Marking_SolidLine,
    Line_Marking_DashedLine,
    Line_Marking_DoubleSolidLine,
    Line_Marking_DoubleDashedLine,
    Line_Marking_LeftSolidRightDashed,
    Line_Marking_RightSolidLeftDashed,
    Line_Marking_DashedBlocks,
    Line_Marking_ShadedArea,
    Line_Marking_PhysicalDivider
};

enum LineMarkingColour {
    Line_Colour_None,
    Line_Colour_Other,
    Line_Colour_White,
    Line_Colour_Yellow,
    Line_Colour_Orange,
    Line_Colour_Red,
    Line_Colour_Blue
};

struct LinearObject
{
    objectID_t id;
    LinearObjectType type;
    LineMarkingType markingType;
    LineMarkingColour markingColour;
    LinearObject(objectID_t _id, LinearObjectType _type, LineMarkingType _markingType,  LineMarkingColour _markingColour)
        :id(_id), type(_type), markingType(_markingType), markingColour(_markingColour){}
};

class LinearObjectProfile : public Profile
{
public:
    const std::vector<LinearObject>& getAllObjs() const { return objs_; }
    virtual ProfileType getType() const {return ProfileType_LinearObjects;}
    
    LinearObjectProfile(uint32_t id, uint16_t pathID, uint32_t offset, uint32_t endOffset) : Profile(id, pathID, offset, endOffset) {}
    void addLinearObject(objectID_t id, LinearObjectType type, LineMarkingType markingType = Line_Marking_Unknown, LineMarkingColour markingColour = Line_Colour_None) { objs_.push_back({id, type, markingType, markingColour}); }

private:
    std::vector<LinearObject> objs_;
    friend class Path;
};

} // namespace RDBVehicleAPI
