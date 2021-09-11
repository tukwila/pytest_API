/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IRoad.h
 * @brief  Definition of Logic extract data structure and IRoad interface.
 *******************************************************************************
 */

#ifndef  IROAD_H_
#define  IROAD_H_

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <opencv2/imgproc.hpp>

#include "typeDef.h"
#include "algoInterface/IServerTransfer.h"

namespace roadDBCore
{

using RoadID_t          = uint64_t;
using LaneID_t          = std::string;
using LineID_t          = std::string;
using CurveID_t         = std::string;

struct NURBS_t;
struct SRoad_t;
struct SLane_t;
struct SLine_t;
struct SCurve_t;
struct SEVP_t;
struct SRoadEdge_t;
struct SRoadDiscreteObject_t;

// maximum lane connection number
const int32_t MAX_CONNECTION_LANE_SIZE = 8;

/**
 *  logic line type
 *  it represents multiple line type such as dash, solid and combined.
 *  also including imputed, paint center line and expect vehicle path.
 *
 */
enum LOGIC_LINE_TYPE_E : uint8_t
{
    LOGIC_LINE_TYPE_SOLID_E = 0,
    LOGIC_LINE_TYPE_DASHED_E,
    LOGIC_LINE_TYPE_IMPUTED_E,
    LOGIC_LINE_TYPE_SLAM_TRACE_E,
    LOGIC_LINE_TYPE_UNLABELED_E,
    LOGIC_LINE_TYPE_DOUBLE_SOLID_E,
    LOGIC_LINE_TYPE_DOUBLE_DASHED_E,
    LOGIC_LINE_TYPE_DASHED_SOLID_E,
    LOGIC_LINE_TYPE_SOLID_DASHED_E,
    LOGIC_LINE_TYPE_DASHED_DASHED_DASHED_E,
    LOGIC_LINE_TYPE_DASHED_SOLID_DASHED_E,
    LOGIC_LINE_TYPE_EVP_E,
    LOGIC_LINE_TYPE_ROAD_EDGE_E,
    LOGIC_LINE_TYPE_MAX_E
};


/**
 * enumeration of lane connection type of all available directions
 *
 */
enum LANE_CONNECTION_E : uint8_t
{
    LANE_CONNECTION_PREDECESSOR_E = 0,
    LANE_CONNECTION_SUCCESSOR_E,
    LANE_CONNECTION_LEFT_E,
    LANE_CONNECTION_RIGHT_E,
    LANE_CONNECTION_MAX_E
};

/**
 * Enumeration definition of EVP attribution
 */
enum EVP_ATTRIBUTE_TYPE_E : uint8_t
{
    EVP_ATTRIBUTE_TYPE_LANE_CHANGABLE_E = 0, // lane change info
    EVP_ATTRIBUTE_TYPE_SPEED_LIMIT_E,        // speed limitation
    EVP_ATTRIBUTE_TYPE_TURN_ATTR_E,          // turn information
    EVP_ATTRIBUTE_TYPE_TS_ATTR_E,          // ts information
    EVP_ATTRIBUTE_TYPE_MAX_E
};

/**
 * Enumeration definition of road access mode
 */
enum DB_ROAD_MODE_E : uint8_t
{
    DB_ROAD_MODE_WHOLE_E = 0,    // The whole road (General info + Edge&Surface details)
    DB_ROAD_MODE_GENERAL_E,      // Only General info of road
    DB_ROAD_MODE_MAX_E
};

enum TRUNCATE_TYPE_E : uint8_t
{
    TRUNCATE_TYPE_NO_E = 0,
    TRUNCATE_TYPE_FIRST_E,
    TRUNCATE_TYPE_SECOND_E,
    TRUNCATE_TYPE_BOTH_E,
    TRUNCATE_TYPE_MAX_E
};

/**
 * Enumeration definition of road mark type
 */
enum ROAD_MARK_TYPE_E : uint8_t
{
    ROAD_MARK_TYPE_NON_CATEGORIZED_E = 0,
    ROAD_MARK_TYPE_PAINT_PED_CROSS_E,
    ROAD_MARK_TYPE_PAINT_HALT_LINE_E,
    ROAD_MARK_TYPE_PAINT_TEMP_PAINT_E,
    ROAD_MARK_TYPE_PAINT_EXCLUSION_E,
    ROAD_MARK_TYPE_PAINT_ARROW_E,
    ROAD_MARK_TYPE_PAINT_BYCICLE_E,
    ROAD_MARK_TYPE_PAINT_BUS_E,
    ROAD_MARK_TYPE_PAINT_HOV_E,
    ROAD_MARK_TYPE_SPEED_BUMP_E,
    ROAD_MARK_TYPE_MAN_HOLE_E,
    ROAD_MARK_TYPE_MAX_E
};

enum VISUALIZATION_TYPE_E : uint8_t
{
    VISUALIZATION_TYPE_UNLABELED_E = 0,
    VISUALIZATION_TYPE_STATIC_E = 1,
    VISUALIZATION_TYPE_ROAD_E = 2,
    VISUALIZATION_TYPE_SIDEWALK_E = 3,
    VISUALIZATION_TYPE_PAINT_E = 4,
    VISUALIZATION_TYPE_BUILDING_E = 5,
    VISUALIZATION_TYPE_WALL_E = 6,
    VISUALIZATION_TYPE_FENCE_E = 7,
    VISUALIZATION_TYPE_CURB_E = 8,
    VISUALIZATION_TYPE_BRIDGE_E = 9,
    VISUALIZATION_TYPE_TUNNEL_E = 10,
    VISUALIZATION_TYPE_POLE_E = 11,
    VISUALIZATION_TYPE_TRAFFICLIGHT_E = 12,
    VISUALIZATION_TYPE_TRAFFICSIGN_E = 13,
    VISUALIZATION_TYPE_VEGETATION_E = 14,
    VISUALIZATION_TYPE_TERRAIN_E = 15,
    VISUALIZATION_TYPE_SKY_E = 16,
    VISUALIZATION_TYPE_PERSON_E = 17,
    VISUALIZATION_TYPE_RIDER_E = 18,
    VISUALIZATION_TYPE_CAR_E = 19,
    VISUALIZATION_TYPE_TRUCK_E = 20,
    VISUALIZATION_TYPE_EGOVEHICLE_E = 21,
    VISUALIZATION_TYPE_VIRTUALWALL_E = 22,
    VISUALIZATION_TYPE_ROADMARK_E = 23,
    VISUALIZATION_TYPE_MAX_E = 24
};

enum INTERSECTION_TYPE_E : uint8_t
{
    INTERSECTION_TYPE_LANE_MERGE_E = 0,
    INTERSECTION_TYPE_LANE_SPLIT_E,
    INTERSECTION_TYPE_ROAD_MERGE_E,
    INTERSECTION_TYPE_ROAD_SPLIT_E,
    INTERSECTION_TYPE_CROSS_E,
    INTERSECTION_TYPE_MAX_E
};

/**
 * Structure dependency  of SRoad_t:
 *    SRoad_t---SLane_t
 *           ------SLine_t
 *           ---------SCurve_t
 *           ------SEVP_t
 *           ---------SLine_t
 *           ---SRoadEdge_t
 *           ------SLine_t
 *           ---SRoadDiscreteObject_t
 *
 */

/**
 *  Base curve structure for lane line
 *  It includes geometry and logic information
 *
 */
struct SCurve_t
{
    std::string                 dbID;
    int32_t                     index;
    uint8_t                     type; //enum LOGIC_LINE_TYPE_E
    uint8_t                     color;//euum LANE_MARKING_COLOR_TYPE_E in RdbV2SCommon.h

    // the width of the paint in the curve
    float32_t                   width;
    float32_t                   length;

    std::shared_ptr<Geometry_t> equationDescription;

    SCurve_t() :
        index(0),
        type(0U),
        color(0U),
        width(0.0F),
        length(0.0F)
    {
    }
};

typedef std::vector<std::shared_ptr<SCurve_t>> vecCurve_t;


/**
 * Data structure of lane line, which only save the left line of the lane;
 * NULL means right line
 *
 */
struct SLine_t
{
    std::string  dbID;
    int32_t      index;  //index in Road;

    float32_t    confidence;
    float32_t    length;

    vecCurve_t   curves;

    SLine_t() :
        index(0),
        confidence(0.0F),
        length(0.0F)
    {
    }
};

typedef std::vector<std::shared_ptr<SLine_t>> vecLine_t;

/**
 * Basic EVP attribute prototype
 *
 */
struct SEVP_Attribute_t
{
    uint8_t      type;         // attribute type, one of EVP_ATTRIBUTE_TYPE_E

    Point3d_t    sPosition;    // EVP attribute start position
    Point3d_t    ePosition;    // EVP attribute end position

    SEVP_Attribute_t(EVP_ATTRIBUTE_TYPE_E attrType = EVP_ATTRIBUTE_TYPE_MAX_E) :
        type(attrType),
        sPosition(0.0, 0.0, 0.0),
        ePosition(0.0, 0.0, 0.0)
    {
    }
};


/**
 * EVP attribute of lane changable
 */
struct SEVP_Attr_Lane_Change_t : public SEVP_Attribute_t
{
    uint8_t      lCanChange;//
    uint8_t      rCanChange;

    SEVP_Attr_Lane_Change_t()
        :SEVP_Attribute_t(EVP_ATTRIBUTE_TYPE_LANE_CHANGABLE_E),
         lCanChange(0),
         rCanChange(0)
    {
    }
};


/**
 * EVP attribute of speed limit
 */
struct SEVP_Attr_Speed_Limit_t : public SEVP_Attribute_t
{
    int32_t      index;        // corresponding traffic sign index in road
    int32_t      objType;      // corresponding to traffic sign objects or surface objects, pending
    float32_t    minSpeed;
    float32_t    maxSpeed;

    SEVP_Attr_Speed_Limit_t()
        :SEVP_Attribute_t(EVP_ATTRIBUTE_TYPE_SPEED_LIMIT_E),
         index(0),
         objType(0),
         minSpeed(0),
         maxSpeed(0)
    {
    }
};


/**
 * EVP attribute of turn information
 */
struct SEVP_Attr_Turn_t : public SEVP_Attribute_t
{
    int32_t      index;        // corresponding arrow index in road
    int32_t      objType;      // corresponding to traffic sign objects or surface objects, pending
    uint8_t      turnType;     // ARROW_PAINT_TYPE_E in RdbV2SRoadObject

    SEVP_Attr_Turn_t()
        :SEVP_Attribute_t(EVP_ATTRIBUTE_TYPE_TURN_ATTR_E),
         index(0),
         objType(0),
         turnType(0)
    {
    }
};

/**
 * EVP attriute if traffic sign info
 */
struct SEVP_Attr_Ts_t : public SEVP_Attribute_t
{
    int32_t      index;
    int32_t      tsType; // roadDBCode in class Definition of traffic sign page 

    SEVP_Attr_Ts_t()
        :SEVP_Attribute_t(EVP_ATTRIBUTE_TYPE_TS_ATTR_E),
        index(0),
        tsType(0)
    {
    }
};

/**
 *  data structure of expected vehicle path
 *  it has a geometry line element and RTV information
 *
 */
struct SEVP_t
{
    // geometry info
    std::shared_ptr<SLine_t>                        line;

    // RTV name and coverage
    std::vector<std::pair<std::string, float32_t>>  vRTVInfo;

    uint64_t                                        fromNodeDBID; //start ID
    uint64_t                                        toNodeDBID; //end ID

    uint32_t                                        type;        // used for various truck type, pending
    uint32_t                                        surfaceType; // paved, not-paved

    uint32_t                                        passedVehicleNum;

    std::vector<std::shared_ptr<SEVP_Attribute_t>>  vSegAttr;    // EVP segment attribute

    SEVP_t() :
        line(nullptr),
        fromNodeDBID(0),
        toNodeDBID(0),
        type(0),
        surfaceType(0),
        passedVehicleNum(0)
    {
    }
};


/**
 * Data structure of discrete (independent) object, include furniture object and surface object
 *
 */
struct SRoadDiscreteObject_t
{
    std::string    dbID;
    int32_t        index;
    int32_t        segmentID;
    uint32_t       type;                    // main type (VISUALIZATION_TYPE_E) + sub type(IRoad Page)
    uint8_t        color;                   // LM_COLOR_E in RdbV2SLandmark.h
    float32_t      locReliability;          // location reliability
    float32_t      semReliability;          // semantic reliability

    Point3f_t      orientation;             // normal vector for visualization
    Point3f_t      centerPoint;             // center point of polygon
    std::vector<Point3f_t>      model;      // for surface object, it is polygon; for furniture object, reserved

    // reserved field for additional information (scientific notation cannot be used)
    std::map<std::string, std::string> additionalInfos;

    SRoadDiscreteObject_t():
            index(0),
            segmentID(0),
            type(0),
            color(0),
            locReliability(0.0F),
            semReliability(0.0F)
    {
    }
};

typedef std::vector<std::shared_ptr<SRoadDiscreteObject_t>> vecRoadDiscreteObject_t;


/**
 * Data structure of road lane which includes left and right lane lane,
 * paint center line and expect vehicle path. Connection with other lanes
 * is also included.
 *
 */
struct SLane_t
{
    std::string                   dbID;
    bool                          isVirtual;
    int32_t                       index;  //index in Road;
    std::shared_ptr<SLine_t>      leftLine;
    std::shared_ptr<SLine_t>      centerLine;
    std::shared_ptr<SLine_t>      rightLine;
    std::vector<std::shared_ptr<SEVP_t>>      evpLine;
    std::vector<std::shared_ptr<SLane_t>>     lLane;
    std::vector<std::shared_ptr<SLane_t>>     rLane;

    SLane_t():
        isVirtual(false),
        index(0),
        leftLine(nullptr),
        centerLine(nullptr),
        rightLine(nullptr)
    {
    }
};

typedef std::vector<std::shared_ptr<SLane_t>>  vecLane_t;


/**
 * Data structure of road edge
 *
 */
struct SRoadEdge_t
{
    std::string              dbID;
    int32_t                  index;  //index in Road;

    bool                     bLeft;
    uint8_t                  type;   //ROAD_EDGE_TYPE_E in RdbV2SRoadObject.h
    float32_t                height;

    std::shared_ptr<SLine_t> line;

    SRoadEdge_t() :
        dbID(""),
        index(0),
        bLeft(true),
        type(0U),
        height(0.0F)
    {
    }
};

typedef std::vector<std::shared_ptr<SRoadEdge_t>>  vecRoadEdge_t;

/**
 * Data structure of road node which is used as the end point of road
 *
 */
struct SRoadNode_t
{
    uint64_t                dbID;
    uint32_t                type; //pending
    Point3d_t               position;     // relative

    SRoadNode_t() :
        dbID(0.0L),
        type(0U)
    {
    }
};

/**
 * Data structure of road plane
 *
 */
struct SNurbsSurface_t
{
    roadDBCore::uint32_t                    pDegree = 3;      //  u direction degree
    roadDBCore::uint32_t                    qDegree = 3;      //  v direction degree
    std::vector<float64_t>                  uKnots;           //  u direction knot
    std::vector<float64_t>                  vKnots;           //  v direction knot
    std::vector<std::vector<Point3f_t>>     ctrlPointMat;
};

/**
 * Data structure of Road which is combined by multiple lanes and edges
 *
 */
struct SRoad_t
{
    // The following fields are general info of SRoad_t
    // roadID
    uint64_t                        dbID;//segment determined by fromNode
    // start(A) and end(B) RoadNode id
    std::shared_ptr<SRoadNode_t>    spFromNode;
    std::shared_ptr<SRoadNode_t>    spToNode;

    // road length computed by trajectory
    float32_t                       length;
    float32_t                       weight;      //the confidence of road
    bool                            fixed;
    std::vector<uint64_t>           divisionIDs;//

    //road plane
    std::shared_ptr<SNurbsSurface_t> spRoadPlane; //current using NURBS_Surface_t

    // to get the reference point, NOT NULL
    vecLane_t                       lanes;

    // the passed segments by this road
    std::set<SegmentID_t>           setPassedSeg;

    vecRoadEdge_t                   roadEdges;

    // The following fields are just used for data processing */
    DB_ROAD_MODE_E                  eMode;

    //The following fields are only be accessed with DB_ROAD_MODE_WHOLE_E mode
    vecRoadDiscreteObject_t         discreteObjects;

    SRoad_t() :
        dbID(0L),
        length(0.0F),
        weight(0.0F),
        fixed(false),
        eMode(DB_ROAD_MODE_WHOLE_E)
    {
    }
};

struct SIntersection_t
{
    uint64_t                                      dbID; //segment determined by first road
    uint8_t                                       type; //INTERSECTION_TYPE_E
    std::vector<std::shared_ptr<SRoad_t>>         roads;
    std::vector<Point3d_t>                        boundary;

    SIntersection_t():
        dbID(0L),
        type(INTERSECTION_TYPE_MAX_E)
    {
    }
};

/**
 *******************************************************************************
 * @class IRoad
 * @brief This class is used to defined interfaces for reading and writing
 *        road data in logicDB.
 *
 *******************************************************************************
 */
class IRoad
{
public:
    virtual ~IRoad(){};
    /**
     *******************************************************************************
     *  @brief get reference segment id
     *
     *  @param [out]  - segID      segment ID
     *
     *  @ret void
     *
     *******************************************************************************
     */
    virtual void getRefSegID(SegmentID_t &segID) = 0;

    /**
     *******************************************************************************
     *  @brief get affected roadID
     *
     *  @param [out]  - affectedInSecIDs   affected intersection IDs
     *
     *  @param [out]  - affectedRoadIDs    affected road IDs
     *
     *  @ret bool
     *
     *******************************************************************************
     */
    virtual bool getAffectedIDs(std::vector<uint64_t> &affectedInSecIDs,
                                std::vector<uint64_t> &affectedRoadIDs) = 0;

    /**
     *******************************************************************************
     *  @brief get passed roadID
     *
     *  @param [out]  - passedInSecIDs   passed intersection IDs
     *
     *  @param [out]  - passedRoadIDs    passed roadIDs
     *
     *  @ret bool
     *
     *******************************************************************************
     */
    virtual bool getPassedIDs(std::vector<uint64_t> &passedInSecIDs,
                              std::vector<uint64_t> &passedRoadIDs) = 0;

    /**
     *******************************************************************************
     *  @brief get SRoad_t according to the roadIDs
     *  Note: 1. it doesn't construct LaneConnection
     *        2. roads contains all road include those of intersections
     *
     *  @param [in]   - intersectionIDs    vector of intersectionIDs that need to be read
     *
     *  @param [in]   - roadIDs            vector of roadIDs that need to be read
     *
     *  @param [out]  - intersections      vector of SIntersection_t
     *
     *  @param [out]  - roads              vector of SRoad_t
     *
     *  @param [in]   - eMode              read mode (default: DB_ROAD_MODE_WHOLE_E)
     *
     *  @ret uint32_t errorCode
     *
     *
     *******************************************************************************
     */
    virtual uint32_t getRoads(const std::vector<uint64_t> &intersectionIDs,
                              const std::vector<uint64_t> &roadIDs,
                              std::vector<std::shared_ptr<SIntersection_t>> &intersections,
                              std::vector<std::shared_ptr<SRoad_t>> &roads,
                              DB_ROAD_MODE_E eMode = DB_ROAD_MODE_WHOLE_E) = 0;

    /**
     *******************************************************************************
     *  @brief save SRoad_t to logicDB
     *  Note: 1. need to construct and save left&right Lane connections
     *        2. the save mode is according to the field eMode of SRoad_t
     *        3. roads contains all roads including those of intersections
     *
     *  @param [out]  - intersections       vector of SIntersection_t
     *
     *  @param [out]  - roads               vector of SRoad_t
     *
     *  @ret uint32_t errorCode
     *
     *
     *******************************************************************************
     */
    virtual uint32_t putRoads(std::vector<std::shared_ptr<SIntersection_t>> &intersections,
                              std::vector<std::shared_ptr<SRoad_t>> &roads) = 0;

    /**
     *******************************************************************************
     *  @brief delete SRoad_t from logicDB
     *
     *  @param [out]  - intersectionIDs       vector id of SIntersection_t
     *
     *  @param [out]  - roadIDs               vector id of SRoad_t
     *
     *  @ret uint32_t errorCode
     *
     *
     *******************************************************************************
     */
    virtual uint32_t deleteRoads(const std::vector<uint64_t> &intersectionIDs,
                                 const std::vector<uint64_t> &roadIDs) = 0;
};

}

#endif
